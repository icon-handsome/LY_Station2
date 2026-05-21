using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using MvCamCtrl.NET;
using System.Runtime.InteropServices;
using System.Threading;
using System.IO;

using System.Drawing.Imaging;
using System.Diagnostics;
using System.Collections.ObjectModel;

namespace InfraredDemo
{
    public partial class InfraredDemo : Form
    {
        [DllImport("kernel32.dll", EntryPoint = "RtlMoveMemory", SetLastError = false)]
        private static extern void CopyMemory(IntPtr dest, IntPtr src, uint count);

        // ch:判断用户自定义像素格式 | en:Determine custom pixel format
        public const Int32 CUSTOMER_PIXEL_FORMAT = unchecked((Int32)0x80000000);
        public const Int32  TEMP_CHUNK_ID_TEST  = 0x00510002;                    // ch:测温区域信息 | en:Temp Measurement region information
        public const Int32  TEMP_CHUNK_ID_ALARM  = 0x00510003 ;                 // ch:测温警告信息 | en:Temp Measurement alarm information
        public const Int32 TEMP_CHUNK_ID_RAW_DATA = 0x00510004 ;               // ch:全屏灰度数据 | en:Full ScreenRaw data
        public const Int32 TEMP_CHUNK_ID_FULL_SCREEN_DATA = 0x00510005 ;       // ch:全屏温度数据 | en:Full Screen Temperature data
        public const Int32  TEMP_CHUNK_ID_MIN_MAX_TEMP = 0x00510006 ;           // ch:最低温最高温 | en:Min Temp & Max Temp
        public const Int32  TEMP_CHUNK_ID_OSD_INFO = unchecked((Int32)0x00510007);               // ch:OSD相关参数 | en:OSD Related parameters

        public const Int32  TEMP_ROI_TYPE_POINT  = 0;
        public const Int32  TEMP_ROI_TYPE_LINE = 1;
        public const Int32  TEMP_ROI_TYPE_POLYGON = 2;
        public const Int32  TEMP_ROI_TYPE_CIRCLE = 3;

        public const Int32  TEMP_ALARM_LEVER_PRE = 0;        // ch:报警等级-预警 | en；Alarm level-Early warning
        public const Int32  TEMP_ALARM_LEVER_WARN = 1;       // ch:报警等级-警告 | en:Alarm level-Warning
        public const Int32  TEMP_ALARM_LEVER_NORMAL = 2;     // ch:报警等级-正常 | en:Alarm level-Normal
        public const Int32  TEMP_ALARM_LEVER_RECOVER = 3;    // ch:报警等级-解除等级 | en:Alarm level-Recover
        public const Int32  TEMP_ALARM_TYPE_MAX  = 0;         // ch:报警类型-最大值 | en:Alarm type-Maximum Temperature Difference
        public const Int32  TEMP_ALARM_TYPE_MIN = 1;         // ch:报警类型-最小值 | en:Alarm type-Minimum Temperature Difference
        public const Int32  TEMP_ALARM_TYPE_AVG  = 2;         // ch:报警类型-平均值 | en:Alarm type-Average Temperature Difference
        public const Int32  TEMP_ALARM_TYPE_DIFFER = 3;      // ch:报警类型-差异值 | en:Alarm type-Variation Temperature Difference
        public const Int32 TEMP_REGION_COUNT = 22;

        // ch:点 | en：Point
        public struct IFR_POINT
        {
            public UInt32 x;
            public UInt32 y;
        }

        // ch:多边形区域 | en:Polygon region
        public struct IFR_POLYGON
        {
            public UInt32 pointNum;              // <ch:多边形实际顶点数 | en:Point of Polygon region
            public UInt32 circleRadius;          // <ch:测温区域圆半径 | en:CircleRadius of Temperature Measurement region
            public IFR_POINT circlepoint;              // <ch:测温区域的圆心点 | en:Circlepoint of Temperature Measurement region

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = TEMP_REGION_COUNT)]
            public IFR_POINT[] pointList;               // <ch:顶点坐标 | en:PointList
        }


        // ch:单个区域测温结果 | en:Temp Measurement Result of Temperature Region
        public struct IFR_OUTCOME_INFO
        {
            public Byte enable;             /// <ch:是否启用：0-否,1-是 | en:Enable: 0-disable,1-Enable
            public Byte regionId;           /// <ch:区域ID | en:RegionId
           
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 34)]
            public Byte[] reserved;
           
            public UInt32 regiontype;         /// <ch:区域类型 0：点 1：线 2：多边形  3:圆 | en:Region Type 0:point 1:Line 2:Polygon

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public Byte[] name;                      /// <ch:区域名称 | en:Region Name

            public UInt32 emissionRate;       /// <ch:发射率: [1，100] | en:Emissivity
            public Int32 minTmp;             /// <ch:最低温度: [-400, 10000]，单位0.1℃ | en:Minimum Temperature: [-400, 10000]，Unit:0.1℃
            public Int32 maxTmp;             /// <ch:最高温度: [-400, 10000]，单位0.1℃ | en:Maximum Temperature: [-400, 10000]，Unit:0.1℃
            public Int32 avrTmp;             /// <ch:平均温度: [-400, 10000]，单位0.1℃ | en:Average Temperature: [-400, 10000]，Unit:0.1℃
            public Int32 diffTmp;            /// <ch:温差： [0, 10400]，单位0.1℃ | en:Temperature variation:[0, 10400],Unit:0.1℃         

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public IFR_POINT[] points;                       /// <ch:保存测试结果中的最高温和最低温坐标，数组下标: 0-最高温，1-最低温 | en:Save Maximum Temperature  and Minimum Temperature coordinates in the test results,Array subscript:0-Maximum Temperature,1-Minimum Temperature

            public IFR_POLYGON polygon;            /// <ch:多边形区域 | en:Polygon region
        }

        public struct IFR_TM_REGION_NUM
        {
            public Byte pointNum;     // ch:点测温个数，最大10个 | en:Number of Point Temperature region,Max:Ten
            public Byte boxNum;       // ch:框测温个数，最大10个 | en:Number of polygon Temperature region,Max:Ten
            public Byte lineNum;      // ch:线测温个数，最多1条 | en:Number of Line Temperature region,Max:One
            public Byte circleNum;    // ch:圆测温个数，最多1个 | en:Number of Circle Temperature region,Max:One
            public Byte total;        // ch:上者之和 | en:All of the above

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public Byte[] res;
        }
        // ch:温度结果列表 | en:Temperature Results List
        public struct IFR_OUTCOME_LIST
        {
            public IFR_TM_REGION_NUM regionNum;                     /// <ch:有效测温区域数量 | en:Number of effective temperature measurement region
           
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public Byte[] res;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = TEMP_REGION_COUNT)]
            public IFR_OUTCOME_INFO[] ifrOutcome;
        }

        // ch:报警上传信息结构体 | en::Uploading information structure of Alarm
        public struct IFR_ALARM_INFO
        {
            public Byte regionId;                     /// <ch:框序号 | en:RegionId
            public Byte alarmkey;                     /// <ch:当前区域报警是否开启开关，上层默认配置开启 | en:Whether the current region alarm is on or off, the upper layer is on by default
            public Byte alarmRule;                    /// <ch:规则:0-大于 1-小于 | en:Rule:0-greater than 1-Less than
            public Byte reserved0;                    /// <ch:预留 | en:Reserve
            public UInt32 regiontype;                    /// <ch:区域类型 0：点 1：线 2：多边形 3:圆 | en:Regiontype 0：point 1：Line 2：Polygon 3:Circle
            public UInt32 alarmType;                     /// <ch:报警类型:0-最高温报警,1-最低温度，2-平均温度,3-温差 | en:AlarmType:0-Maximum Temperature alarm,1-Minimum Temperature,2-Average Temperature,3-Temperature variation
            public UInt32 alarmLevel;                    /// <ch:报警级别：0-预警，1-报警，2-正常，3-解除预警 | en:Alarm Level:0-Early waining,2-Normal,3-Cancel early warning
           
            public Int32 measureTmpData;                         /// <ch:测量值，单位0.1℃ | en:Temperature Measurements,Unit0.1℃
            public Int32 ruleTmpData;                            /// <ch:规则设定值，单位0.1℃ | en:Rule Settings，Unit0.1℃
            public IFR_POLYGON polygon;                        /// <ch:区域位置 | en:Region Location

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public IFR_POINT[] points;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public Byte[] reserved1;
        }


        // ch:区域温差报警上传信息结构体 | en:Uploading information structure of  Multi Temperature Region Alarm
        public struct IFR_DIFF_ALARM_INFO
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public Byte[] regionSet;

            public Byte alarmkey;                     /// <ch:当前区域报警是否开启开关 | en:Whether the current region alarm is on
            public Byte reserved0;                    /// <ch:预留 | en:Reserve
            public UInt32 alarmType;                     /// <ch:报警类型:0-最高温报警,1-最低温度，2-平均温度,3-温差 | en:AlarmType:0-Maximum Temperature alarm,1-Minimum Temperature,2-Average Temperature,3-Temperature variation
            public Byte alarmRule;                    /// <ch:规则:0-大于 1-小于 | en:Rule:0-greater than 1-Less than

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
            public Byte[] reserved1;

            public UInt32 alarmLevel;                    /// <ch:报警级别：0-预警，1-报警，2-正常，3-解除预警 | en:Alarm Level:0-Early waining,2-Normal,3-Cancel early warning

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
            public UInt32[] measureTmpData;

            public Int32 ruleTmpData;                            /// <ch:规则设定值，单位0.1℃ | en:Rule Settings，Unit0.1℃                             

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public Byte[] reserved2;
        }


        public struct IFR_ALARM_UPLOAD_INFO
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = TEMP_REGION_COUNT)]
            public IFR_ALARM_INFO[] alarmOutcome;            /// ch:单区域报警结果 | en:Temperature Region Alarm result

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public IFR_DIFF_ALARM_INFO[] alarmDiffOutcome;   /// ch:区域间温差报警结果 | en: Multi Temperature Region Alarm result    
        }
        
        // ch:全屏温度信息 | en:Full Screen Temperature information
        public struct IFR_FULL_SCREEN_MAX_MIN_INFO
        {
            public UInt32 nMaxTemp;
            public UInt32 nMinTemp;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public Byte[] reserved;
        }

        // ch:测温区域显示规则 | en:Temperature region Display Rule
        public struct IRF_REGION_DISP_INFO
        {
            public UInt32 regionDispIndex;                    /**< ch:区域显示规则索引号 | en:Region Display rule Index number */
            public UInt32 regionDispEnable;                   /**< ch:测温区域显示使能 | en: Temp Measurement region Display Enable */
            public UInt32 regionMaxTempDispEnable;            /**< ch:区域最大温度显示使能 | en:Region Maximum Temperature Display Enable */
            public UInt32 regionMinTempDispEnable;            /**< ch:区域最小温度显示使能 | en:Region Minimum  Temperature Display Enable*/
            public UInt32 regionAvgTempDispEnable;            /**< ch:区域平均温度显示使能 | en:Region Average Temperature Display Enable*/
            public UInt32 regionAlarmDispEnable;              /**< ch:区域告警状态显示使能 | en:Region Alarm Temperature Display Enable*/
        }

        // ch:区域测温图像叠加特征信息结构体 | en:Information Structure of superimposed feature  of region temperature measurement image
        public struct IRF_OSD_INFO
        {
            public UInt32 legendDisplayEnable;    /**< ch:图例（温度条）是否使能 1 使能 0 不使能 | en:Legend: 1:Enable 0:Disable*/
            public UInt32 osdProcessor;           /**< ch:红外相机OSD叠加控制器选择 0 无叠加 1 相机叠加 2 客户端叠加 | en:Over Screen Display Processor: 0:None 1:Camera 2:Client*/

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = TEMP_REGION_COUNT)]
            public IRF_REGION_DISP_INFO[] regionDispRules;   /// ch:区域间温差报警结果 | en: Multi Temperature Region Alarm result  
        } 
 
        


        MyCamera.MV_CC_DEVICE_INFO_LIST m_stDeviceList = new MyCamera.MV_CC_DEVICE_INFO_LIST();
        private MyCamera m_MyCamera = new MyCamera();
        bool m_bGrabbing = false;
        Thread m_hReceiveThread = null;
        MyCamera.MV_FRAME_OUT_INFO_EX m_stFrameInfo = new MyCamera.MV_FRAME_OUT_INFO_EX();

        // ch:用于从驱动获取图像的缓存 | en:Buffer for getting image from driver
        UInt32 m_nBufSizeForDriver = 0;
        IntPtr m_BufForDriver = IntPtr.Zero;
        private static Object BufForDriverLock = new Object();

        // ch:Bitmap及其像素格式 | en:Bitmap and Pixel Format
        Bitmap m_bitmap = null;
        PixelFormat m_bitmapPixelFormat = PixelFormat.DontCare;
        IntPtr m_ConvertDstBuf = IntPtr.Zero;
        UInt32 m_nConvertDstBufLen = 0;

        IntPtr displayHandle;

        bool m_bOpenDevice;                        // ch:是否打开设备 | en:Whether to open device
        bool m_bStartGrabbing;                     // ch:是否开始抓图 | en:Whether to start grabbing

        public InfraredDemo()
        {
            InitializeComponent();
            m_bOpenDevice = false;
            m_bStartGrabbing = false;
            EnableControls(false);
            this.Load += new EventHandler(this.InfraredDemo_Load);
        }

        private void InfraredDemo_Load(object sender, EventArgs e)
        {
            // ch: 初始化 SDK | en: Initialize SDK
            MyCamera.MV_CC_Initialize_NET();
        }

        public static FormRegionSetting RegionSettingForm = null;   // 区域设置界面
        public static FormAlarmSetting AlarmSettingForm = null;   // 告警设置界面

        // ch:显示错误信息 | en:Show error message
        private void ShowErrorMsg(string csMessage, int nErrorNum)
        {
            string errorMsg;
            if (nErrorNum == 0)
            {
                errorMsg = csMessage;
            }
            else
            {
                errorMsg = csMessage + ": Error =" + String.Format("{0:X}", nErrorNum);
            }

            switch (nErrorNum)
            {
                case MyCamera.MV_E_HANDLE: errorMsg += "Error or invalid handle "; break;
                case MyCamera.MV_E_SUPPORT: errorMsg += "Not supported function "; break;
                case MyCamera.MV_E_BUFOVER: errorMsg += "Cache is full "; break;
                case MyCamera.MV_E_CALLORDER: errorMsg += "Function calling order error "; break;
                case MyCamera.MV_E_PARAMETER: errorMsg += "Incorrect parameter "; break;
                case MyCamera.MV_E_RESOURCE: errorMsg += "Applying resource failed "; break;
                case MyCamera.MV_E_NODATA: errorMsg += "No data "; break;
                case MyCamera.MV_E_PRECONDITION: errorMsg += "Precondition error, or running environment changed "; break;
                case MyCamera.MV_E_VERSION: errorMsg += "Version mismatches "; break;
                case MyCamera.MV_E_NOENOUGH_BUF: errorMsg += "Insufficient memory "; break;
                case MyCamera.MV_E_ABNORMAL_IMAGE: errorMsg += "Abnormal image, maybe incomplete image because of lost packet "; break;
                case MyCamera.MV_E_UNKNOW: errorMsg += "Unknown error "; break;
                case MyCamera.MV_E_GC_GENERIC: errorMsg += "General error "; break;
                case MyCamera.MV_E_GC_ACCESS: errorMsg += "Node accessing condition error "; break;
                case MyCamera.MV_E_ACCESS_DENIED: errorMsg += "No permission "; break;
                case MyCamera.MV_E_BUSY: errorMsg += "Device is busy, or network disconnected "; break;
                case MyCamera.MV_E_NETER: errorMsg += "Network error "; break;
            }

            MessageBox.Show(errorMsg, "PROMPT");
        }

        private void bnEnum_Click(object sender, EventArgs e)
        {
            DeviceListAcq();
            EnableControls(true);
        }

        private void DeviceListAcq()
        {
            // ch:创建设备列表 | en:Create Device List
            System.GC.Collect();
            cbDeviceList.Items.Clear();
            m_stDeviceList.nDeviceNum = 0;
            int nRet = MyCamera.MV_CC_EnumDevices_NET(MyCamera.MV_GIGE_DEVICE | MyCamera.MV_USB_DEVICE, ref m_stDeviceList);
            if (0 != nRet)
            {
                ShowErrorMsg("Enumerate devices fail!", 0);
                return;
            }

            // ch:在窗体列表中显示设备名 | en:Display device name in the form list
            for (int i = 0; i < m_stDeviceList.nDeviceNum; i++)
            {
                MyCamera.MV_CC_DEVICE_INFO device = (MyCamera.MV_CC_DEVICE_INFO)Marshal.PtrToStructure(m_stDeviceList.pDeviceInfo[i], typeof(MyCamera.MV_CC_DEVICE_INFO));
                string strUserDefinedName = "";
                if (device.nTLayerType == MyCamera.MV_GIGE_DEVICE)
                {
                    MyCamera.MV_GIGE_DEVICE_INFO_EX gigeInfo = (MyCamera.MV_GIGE_DEVICE_INFO_EX)MyCamera.ByteToStruct(device.SpecialInfo.stGigEInfo, typeof(MyCamera.MV_GIGE_DEVICE_INFO_EX));

                    if ((gigeInfo.chUserDefinedName.Length > 0) && (gigeInfo.chUserDefinedName[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(gigeInfo.chUserDefinedName))
                        {
                            strUserDefinedName = Encoding.UTF8.GetString(gigeInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        else
                        {
                            strUserDefinedName = Encoding.Default.GetString(gigeInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        cbDeviceList.Items.Add("GEV: " + strUserDefinedName + " (" + gigeInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cbDeviceList.Items.Add("GEV: " + gigeInfo.chManufacturerName + " " + gigeInfo.chModelName + " (" + gigeInfo.chSerialNumber + ")");
                    }
                }
                else if (device.nTLayerType == MyCamera.MV_USB_DEVICE)
                {
                    MyCamera.MV_USB3_DEVICE_INFO_EX usbInfo = (MyCamera.MV_USB3_DEVICE_INFO_EX)MyCamera.ByteToStruct(device.SpecialInfo.stUsb3VInfo, typeof(MyCamera.MV_USB3_DEVICE_INFO_EX));

                    if ((usbInfo.chUserDefinedName.Length > 0) && (usbInfo.chUserDefinedName[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(usbInfo.chUserDefinedName))
                        {
                            strUserDefinedName = Encoding.UTF8.GetString(usbInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        else
                        {
                            strUserDefinedName = Encoding.Default.GetString(usbInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        cbDeviceList.Items.Add("U3V: " + strUserDefinedName + " (" + usbInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cbDeviceList.Items.Add("U3V: " + usbInfo.chManufacturerName + " " + usbInfo.chModelName + " (" + usbInfo.chSerialNumber + ")");
                    }
                }
            }

            // ch:选择第一项 | en:Select the first item
            if (m_stDeviceList.nDeviceNum != 0)
            {
                cbDeviceList.SelectedIndex = 0;
            }
        }

        public int SetEnumIntoCombo(string strKey, ref ComboBox ctrlComboBox)
        {
            string str = ctrlComboBox.SelectedItem.ToString();
            MyCamera.MVCC_ENUMENTRY stEnumInfo = new MyCamera.MVCC_ENUMENTRY();
            MyCamera.MVCC_ENUMVALUE stEnumValue = new MyCamera.MVCC_ENUMVALUE();
            int nRet = m_MyCamera.MV_CC_GetEnumValue_NET(strKey, ref stEnumValue);
            if (MyCamera.MV_OK != nRet)
            {
                return nRet;
            }
            for (int i = 0; i < stEnumValue.nSupportedNum; ++i)
            {
                stEnumInfo.nValue = stEnumValue.nSupportValue[i];
                nRet = m_MyCamera.MV_CC_GetEnumEntrySymbolic_NET(strKey, ref stEnumInfo);
                if (MyCamera.MV_OK == nRet && str.Equals(Encoding.Default.GetString(stEnumInfo.chSymbolic), StringComparison.OrdinalIgnoreCase))
                {
                    nRet = m_MyCamera.MV_CC_SetEnumValue_NET(strKey, stEnumInfo.nValue);
                    if (MyCamera.MV_OK != nRet)
                    {
                        return nRet;
                    }
                    break;
                }
            }
            return nRet;
        }

        private void bnOpen_Click(object sender, EventArgs e)
        {
            if (m_stDeviceList.nDeviceNum == 0 || cbDeviceList.SelectedIndex == -1)
            {
                ShowErrorMsg("No device, please select", 0);
                return;
            }

            // ch:获取选择的设备信息 | en:Get selected device information
            MyCamera.MV_CC_DEVICE_INFO device =
                (MyCamera.MV_CC_DEVICE_INFO)Marshal.PtrToStructure(m_stDeviceList.pDeviceInfo[cbDeviceList.SelectedIndex],
                                                              typeof(MyCamera.MV_CC_DEVICE_INFO));

            // ch:打开设备 | en:Open device
            if (null == m_MyCamera)
            {
                m_MyCamera = new MyCamera();
                if (null == m_MyCamera)
                {
                    ShowErrorMsg("Applying resource fail!", MyCamera.MV_E_RESOURCE);
                    return;
                }
            }

            int nRet = m_MyCamera.MV_CC_CreateDevice_NET(ref device);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Create device fail!", nRet);
                return;
            }

            nRet = m_MyCamera.MV_CC_OpenDevice_NET();
            if (MyCamera.MV_OK != nRet)
            {
                m_MyCamera.MV_CC_DestroyDevice_NET();
                ShowErrorMsg("Device open fail!", nRet);
                return;
            }

            m_MyCamera.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_OFF);

            m_bOpenDevice = true;
            EnableControls(true);
            bnGetParameter_Click(null, null);
        }

        private void EnableControls(bool bIsCameraReady)
        {
            bnOpen.Enabled = (m_bOpenDevice ? false : (bIsCameraReady ? true : false));
            bnClose.Enabled = ((m_bOpenDevice && bIsCameraReady) ? true : false);
            bnStartGrab.Enabled = ((m_bStartGrabbing && bIsCameraReady) ? false : (m_bOpenDevice ? true : false));
            bnStopGrab.Enabled = (m_bStartGrabbing ? true : false);
            cbPixelFormat.Enabled = ((m_bStartGrabbing && bIsCameraReady) ? false : (m_bOpenDevice ? true : false));
            cbDisplaySource.Enabled = (m_bOpenDevice ? true : false);
            cbLegendCheck.Enabled = (m_bOpenDevice ? true : false);
            cbRegionSelect.Enabled = (m_bOpenDevice ? true : false);
            bnRegionSetting.Enabled = (m_bOpenDevice ? true : false);
            bnWarningSetting.Enabled = (m_bOpenDevice ? true : false);
            teTransmissivity.Enabled = (m_bOpenDevice ? true : false);
            teTargetDistance.Enabled = (m_bOpenDevice ? true : false);
            teEmissivity.Enabled = (m_bOpenDevice ? true : false);
            cbMeasureRange.Enabled = (m_bOpenDevice ? true : false);
            bnGetParameter.Enabled = (m_bOpenDevice ? true : false);
            bnSetParameter.Enabled = (m_bOpenDevice ? true : false);
            cbPaletteMode.Enabled = (m_bOpenDevice ? true : false);
            cbExportModeCheck.Enabled = (m_bOpenDevice ? true : false);
        }

        private int ReadEnumIntoCombo(string strKey, ref ComboBox ctrlComboBox)
        {
            MyCamera.MVCC_ENUMENTRY stEnumInfo = new MyCamera.MVCC_ENUMENTRY();
            MyCamera.MVCC_ENUMVALUE stEnumValue = new MyCamera.MVCC_ENUMVALUE();
            int nRet = m_MyCamera.MV_CC_GetEnumValue_NET(strKey, ref stEnumValue);
            if (MyCamera.MV_OK != nRet)
            {
                return nRet;
            }
            ctrlComboBox.Items.Clear();
            int nIndex = -1;
            for (int i = 0; i < stEnumValue.nSupportedNum; ++i)
            {
                stEnumInfo.nValue = stEnumValue.nSupportValue[i];
                nRet = m_MyCamera.MV_CC_GetEnumEntrySymbolic_NET(strKey, ref stEnumInfo);
                if (MyCamera.MV_OK == nRet)
                {
                    ctrlComboBox.Items.Add(Encoding.Default.GetString(stEnumInfo.chSymbolic));
                }
                if (stEnumInfo.nValue == stEnumValue.nCurValue)
                {
                    nIndex = ctrlComboBox.FindString(Encoding.Default.GetString(stEnumInfo.chSymbolic));
                }
                if (nIndex >= 0)
                {
                    ctrlComboBox.SelectedIndex = nIndex;
                }
            }
            return MyCamera.MV_OK;
        }

        private void bnGetParameter_Click(object sender, EventArgs e)
        {
            int nRet = ReadEnumIntoCombo("PixelFormat", ref cbPixelFormat);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get PixelFormat Fail!", nRet);
                return;
            }

            nRet = ReadEnumIntoCombo("OverScreenDisplayProcessor", ref cbDisplaySource);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get OverScreenDisplayProcessor Fail!", nRet);
                return;
            }

            nRet = ReadEnumIntoCombo("PalettesMode", ref cbPaletteMode);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get PalettesMode Fail!", nRet);
                return;
            }

            bool bValue = false;
            nRet = m_MyCamera.MV_CC_GetBoolValue_NET("LegendDisplayEnable", ref bValue);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get LegendDisplayEnable Fail!", nRet);
                return;
            }
            else
            {
                cbLegendCheck.Checked = bValue;
            }

            nRet = m_MyCamera.MV_CC_GetBoolValue_NET("MtExpertMode", ref bValue);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get MtExpertMode Fail!", nRet);
                return;
            }
            else
            {
                cbExportModeCheck.Checked = bValue;
            }

            nRet = ReadEnumIntoCombo("TempRegionSelector", ref cbRegionSelect);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get TempRegionSelector Fail!", nRet);
                return;
            }

            nRet = ReadEnumIntoCombo("TempMeasurementRange", ref cbMeasureRange);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get TempMeasurementRange Fail!", nRet);
                return;
            }

            MyCamera.MVCC_INTVALUE_EX oIntValue = new MyCamera.MVCC_INTVALUE_EX();
            nRet = m_MyCamera.MV_CC_GetIntValueEx_NET("AtmosphericTransmissivity", ref oIntValue);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get AtmosphericTransmissivity Fail!", nRet);
            }
            else
            {
                teTransmissivity.Text = oIntValue.nCurValue.ToString();
            }

            MyCamera.MVCC_FLOATVALUE oFloatValue = new MyCamera.MVCC_FLOATVALUE();
            nRet = m_MyCamera.MV_CC_GetFloatValue_NET("TargetDistance", ref oFloatValue);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get TargetDistance Fail!", nRet);
                return;
            }
            else
            {
                teTargetDistance.Text = oFloatValue.fCurValue.ToString();
            }

            nRet = m_MyCamera.MV_CC_GetFloatValue_NET("FullScreenEmissivity", ref oFloatValue);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get FullScreenEmissivity!", nRet);
                return;
            }
            else
            {
                teEmissivity.Text = oFloatValue.fCurValue.ToString();
            }
        }

        // ch:像素类型是否为Mono格式 | en:If Pixel Type is Mono 
        private Boolean IsMono(UInt32 enPixelType)
        {
            switch (enPixelType)
            {
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono1p:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono2p:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono4p:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono8:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono8_Signed:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono10:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono10_Packed:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono12:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono12_Packed:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono14:
                case (UInt32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono16:
                    return true;
                default:
                    return false;
            }
        }

        // ch:取图前的必要操作步骤 | en:Necessary operation before grab
        private Int32 NecessaryOperBeforeGrab()
        {
            // ch:取图像宽 | en:Get Iamge Width
            MyCamera.MVCC_INTVALUE_EX stWidth = new MyCamera.MVCC_INTVALUE_EX();
            int nRet = m_MyCamera.MV_CC_GetIntValueEx_NET("Width", ref stWidth);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get Width Info Fail!", nRet);
                return nRet;
            }
            // ch:取图像高 | en:Get Iamge Height
            MyCamera.MVCC_INTVALUE_EX stHeight = new MyCamera.MVCC_INTVALUE_EX();
            nRet = m_MyCamera.MV_CC_GetIntValueEx_NET("Height", ref stHeight);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get Height Info Fail!", nRet);
                return nRet;
            }
            // ch:取像素格式 | en:Get Pixel Format
            MyCamera.MVCC_ENUMVALUE stPixelFormat = new MyCamera.MVCC_ENUMVALUE();
            nRet = m_MyCamera.MV_CC_GetEnumValue_NET("PixelFormat", ref stPixelFormat);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get Pixel Format Fail!", nRet);
                return nRet;
            }

            // ch:设置bitmap像素格式，申请相应大小内存 | en:Set Bitmap Pixel Format, alloc memory
            if ((Int32)MyCamera.MvGvspPixelType.PixelType_Gvsp_Undefined == stPixelFormat.nCurValue)
            {
                ShowErrorMsg("Unknown Pixel Format!", MyCamera.MV_E_UNKNOW);
                return MyCamera.MV_E_UNKNOW;
            }
            else if (IsMono(stPixelFormat.nCurValue))
            {
                m_bitmapPixelFormat = PixelFormat.Format8bppIndexed;

                if (IntPtr.Zero != m_ConvertDstBuf)
                {
                    Marshal.Release(m_ConvertDstBuf);
                    m_ConvertDstBuf = IntPtr.Zero;
                }

                // Mono8为单通道
                m_nConvertDstBufLen = (UInt32)(stWidth.nCurValue * stHeight.nCurValue);
                m_ConvertDstBuf = Marshal.AllocHGlobal((Int32)m_nConvertDstBufLen);
                if (IntPtr.Zero == m_ConvertDstBuf)
                {
                    ShowErrorMsg("Malloc Memory Fail!", MyCamera.MV_E_RESOURCE);
                    return MyCamera.MV_E_RESOURCE;
                }
            }
            else
            {
                m_bitmapPixelFormat = PixelFormat.Format24bppRgb;

                if (IntPtr.Zero != m_ConvertDstBuf)
                {
                    Marshal.FreeHGlobal(m_ConvertDstBuf);
                    m_ConvertDstBuf = IntPtr.Zero;
                }

                // RGB为三通道
                m_nConvertDstBufLen = (UInt32)(3 * stWidth.nCurValue * stHeight.nCurValue);
                m_ConvertDstBuf = Marshal.AllocHGlobal((Int32)m_nConvertDstBufLen);
                if (IntPtr.Zero == m_ConvertDstBuf)
                {
                    ShowErrorMsg("Malloc Memory Fail!", MyCamera.MV_E_RESOURCE);
                    return MyCamera.MV_E_RESOURCE;
                }
            }

            // 确保释放保存了旧图像数据的bitmap实例，用新图像宽高等信息new一个新的bitmap实例
            if (null != m_bitmap)
            {
                m_bitmap.Dispose();
                m_bitmap = null;
            }
            m_bitmap = new Bitmap((Int32)stWidth.nCurValue, (Int32)stHeight.nCurValue, m_bitmapPixelFormat);

            // ch:Mono8格式，设置为标准调色板 | en:Set Standard Palette in Mono8 Format
            if (PixelFormat.Format8bppIndexed == m_bitmapPixelFormat)
            {
                ColorPalette palette = m_bitmap.Palette;
                for (int i = 0; i < palette.Entries.Length; i++)
                {
                    palette.Entries[i] = Color.FromArgb(i, i, i);
                }
                m_bitmap.Palette = palette;
            }

            return MyCamera.MV_OK;
        }

        public void ReceiveThreadProcess()
        {
            MyCamera.MV_FRAME_OUT stFrameInfo = new MyCamera.MV_FRAME_OUT();
            MyCamera.MV_DISPLAY_FRAME_INFO stDisplayInfo = new MyCamera.MV_DISPLAY_FRAME_INFO();
            MyCamera.MV_PIXEL_CONVERT_PARAM stConvertInfo = new MyCamera.MV_PIXEL_CONVERT_PARAM();
            int nRet = MyCamera.MV_OK;

            using (StreamWriter writer = new StreamWriter("InfraredLog.txt", true ))
            {

                while (m_bGrabbing)
                {
                    nRet = m_MyCamera.MV_CC_GetImageBuffer_NET(ref stFrameInfo, 1000);
                    if (nRet == MyCamera.MV_OK)
                    {
                        lock (BufForDriverLock)
                        {
                            if (m_BufForDriver == IntPtr.Zero || stFrameInfo.stFrameInfo.nFrameLen > m_nBufSizeForDriver)
                            {
                                if (m_BufForDriver != IntPtr.Zero)
                                {
                                    Marshal.Release(m_BufForDriver);
                                    m_BufForDriver = IntPtr.Zero;
                                }

                                m_BufForDriver = Marshal.AllocHGlobal((Int32)stFrameInfo.stFrameInfo.nFrameLen);
                                if (m_BufForDriver == IntPtr.Zero)
                                {
                                    return;
                                }
                                m_nBufSizeForDriver = stFrameInfo.stFrameInfo.nFrameLen;
                            }

                            m_stFrameInfo = stFrameInfo.stFrameInfo;
                            CopyMemory(m_BufForDriver, stFrameInfo.pBufAddr, stFrameInfo.stFrameInfo.nFrameLen);

                            // ch:转换像素格式 | en:Convert Pixel Format
                            stConvertInfo.nWidth = stFrameInfo.stFrameInfo.nWidth;
                            stConvertInfo.nHeight = stFrameInfo.stFrameInfo.nHeight;
                            stConvertInfo.enSrcPixelType = stFrameInfo.stFrameInfo.enPixelType;
                            stConvertInfo.pSrcData = stFrameInfo.pBufAddr;
                            stConvertInfo.nSrcDataLen = stFrameInfo.stFrameInfo.nFrameLen;
                            stConvertInfo.pDstBuffer = m_ConvertDstBuf;
                            stConvertInfo.nDstBufferSize = m_nConvertDstBufLen;
                            if (PixelFormat.Format8bppIndexed == m_bitmap.PixelFormat)
                            {
                                stConvertInfo.enDstPixelType = MyCamera.MvGvspPixelType.PixelType_Gvsp_Mono8;
                                m_MyCamera.MV_CC_ConvertPixelType_NET(ref stConvertInfo);
                            }
                            else
                            {
                                stConvertInfo.enDstPixelType = MyCamera.MvGvspPixelType.PixelType_Gvsp_BGR8_Packed;
                                m_MyCamera.MV_CC_ConvertPixelType_NET(ref stConvertInfo);
                            }

                            // ch:保存Bitmap数据 | en:Save Bitmap Data
                            BitmapData bitmapData = m_bitmap.LockBits(new Rectangle(0, 0, stConvertInfo.nWidth, stConvertInfo.nHeight), ImageLockMode.ReadWrite, m_bitmap.PixelFormat);
                            CopyMemory(bitmapData.Scan0, stConvertInfo.pDstBuffer, (UInt32)(bitmapData.Stride * m_bitmap.Height));
                            m_bitmap.UnlockBits(bitmapData);
                        }

                        stDisplayInfo.hWnd = displayHandle;
                        stDisplayInfo.pData = stFrameInfo.pBufAddr;
                        stDisplayInfo.nDataLen = stFrameInfo.stFrameInfo.nFrameLen;
                        stDisplayInfo.nWidth = stFrameInfo.stFrameInfo.nWidth;
                        stDisplayInfo.nHeight = stFrameInfo.stFrameInfo.nHeight;
                        stDisplayInfo.enPixelType = stFrameInfo.stFrameInfo.enPixelType;
                        m_MyCamera.MV_CC_DisplayOneFrame_NET(ref stDisplayInfo);


                        IFR_OUTCOME_LIST stOutComeList = new IFR_OUTCOME_LIST();
                        IFR_ALARM_UPLOAD_INFO stAlarmInfoList = new IFR_ALARM_UPLOAD_INFO();
                        IFR_FULL_SCREEN_MAX_MIN_INFO stFullScreenMaxMin = new IFR_FULL_SCREEN_MAX_MIN_INFO();
                        IRF_OSD_INFO stOsdInfo = new IRF_OSD_INFO();


                        MyCamera.MV_CHUNK_DATA_CONTENT stChunkData = new MyCamera.MV_CHUNK_DATA_CONTENT();
                        int mvChunkInfoSize = Marshal.SizeOf(stChunkData);


                        for (int i = 0; i < stFrameInfo.stFrameInfo.nUnparsedChunkNum; ++i)
                        {
                            stChunkData = (MyCamera.MV_CHUNK_DATA_CONTENT)Marshal.PtrToStructure((IntPtr)(stFrameInfo.stFrameInfo.UnparsedChunkList.pUnparsedChunkContent.ToInt64() + i * mvChunkInfoSize), typeof(MyCamera.MV_CHUNK_DATA_CONTENT));

                            if (TEMP_CHUNK_ID_TEST == stChunkData.nChunkID &&
                                Marshal.SizeOf(stOutComeList) == stChunkData.nChunkLen)
                            {
                                stOutComeList = (IFR_OUTCOME_LIST)Marshal.PtrToStructure(stChunkData.pChunkData, typeof(IFR_OUTCOME_LIST));
                            }
                            else if (TEMP_CHUNK_ID_ALARM == stChunkData.nChunkID &&
                                    Marshal.SizeOf(stAlarmInfoList) == stChunkData.nChunkLen)
                            {
                                stAlarmInfoList = (IFR_ALARM_UPLOAD_INFO)Marshal.PtrToStructure(stChunkData.pChunkData, typeof(IFR_ALARM_UPLOAD_INFO));
                            }
                            else if (TEMP_CHUNK_ID_MIN_MAX_TEMP == stChunkData.nChunkID &&
                                    Marshal.SizeOf(stFullScreenMaxMin) == stChunkData.nChunkLen)
                            {
                                stFullScreenMaxMin = (IFR_FULL_SCREEN_MAX_MIN_INFO)Marshal.PtrToStructure(stChunkData.pChunkData, typeof(IFR_FULL_SCREEN_MAX_MIN_INFO));
                            }
                            else if (TEMP_CHUNK_ID_OSD_INFO == stChunkData.nChunkID &&
                                    Marshal.SizeOf(stOsdInfo) == stChunkData.nChunkLen)
                            {
                                stOsdInfo = (IRF_OSD_INFO)Marshal.PtrToStructure(stChunkData.pChunkData, typeof(IRF_OSD_INFO));
                            }
                        }

                        writer.WriteLine("************Beginning Output Test Temperature Info, Frame Number:{0}************", stFrameInfo.stFrameInfo.nFrameNum);

                        IRF_REGION_DISP_INFO stRegionDispRules = new IRF_REGION_DISP_INFO();
                        int nRegionDispRulesSize = Marshal.SizeOf(stRegionDispRules);

                        for (int i = 0; i < TEMP_REGION_COUNT; ++i)
                        {
                            if (stOsdInfo.regionDispRules == null)
                            {
                                continue;
                            }

                            stRegionDispRules = stOsdInfo.regionDispRules[i];
                            //stRegionDispRules = (IRF_REGION_DISP_INFO)Marshal.PtrToStructure(stOsdInfo.regionDispRules[i], typeof(IRF_REGION_DISP_INFO));
                            if (stRegionDispRules.regionDispEnable.Equals(0))
                            {
                                continue;
                            }


                            if (stAlarmInfoList.alarmOutcome == null)
                            {
                                continue;
                            }
                            IFR_ALARM_INFO stAlarmInfo = stAlarmInfoList.alarmOutcome[i];

                            //deal with the alarm region.
                            if (stAlarmInfo.alarmkey > 0 && TEMP_ALARM_LEVER_WARN == stAlarmInfo.alarmLevel &&
                                (false == stRegionDispRules.regionAlarmDispEnable.Equals(0)))
                            {
                                IFR_POINT stPoint0 = stAlarmInfo.points[0];
                                IFR_POINT stPoint1 = stAlarmInfo.points[1];

                                if (TEMP_ALARM_TYPE_MAX == stAlarmInfo.alarmType)
                                {

                                    writer.WriteLine("RegionID: {0}; Alarm Max Temp : {1}; PointX : {2}; PointY : {3}",
                                    stAlarmInfo.regionId, 0.1 * stAlarmInfo.measureTmpData, stPoint0.x, stPoint0.y);

                                }
                                else if (TEMP_ALARM_TYPE_MIN == stAlarmInfo.alarmType)
                                {

                                    writer.WriteLine("RegionID: {0}; Alarm Min Temp : {1}; PointX : {2}; PointY :{3}",
                                    stAlarmInfo.regionId, 0.1 * stAlarmInfo.measureTmpData, stPoint1.x, stPoint1.y);
                                }
                                else if (TEMP_ALARM_TYPE_AVG == stAlarmInfo.alarmType)
                                {

                                    writer.WriteLine("RegionID: {0}; Alarm Avg Temp : {1}; PointX :{2}; PointY : {3}",
                                    stAlarmInfo.regionId, 0.1 * stAlarmInfo.measureTmpData, stPoint0.x, stPoint0.y);
                                }
                                else
                                {
                                    writer.WriteLine("RegionID: {0}; Alarm Differ Temp : {1}; PointX : {2}; PointY : {3}",
                                    stAlarmInfo.regionId, 0.1 * stAlarmInfo.measureTmpData, stPoint0.x, stPoint0.y);
                                }

                                continue;
                            }

                            //deal with the enable region. 
                            IFR_OUTCOME_INFO stOutCome = stOutComeList.ifrOutcome[i];
                            if (stOutCome.enable.Equals(0))
                            {
                                continue;
                            }

                            IFR_POINT stOutComePoint0 = stOutCome.points[0];
                            IFR_POINT stOutComePoint1 = stOutCome.points[1];

                            if (TEMP_ROI_TYPE_POINT == stOutCome.regiontype)
                            {
                                if (false == stRegionDispRules.regionAvgTempDispEnable.Equals(0))
                                {
                                    writer.WriteLine("RegionID: {0}; Avg Temp : {1}; PointX : {2}; PointY :{3}",
                                    stOutCome.regionId, 0.1 * stOutCome.avrTmp, stOutComePoint0.x, stOutComePoint0.y);
                                }
                            }
                            else
                            {
                                //均值显示在最大值的后面
                                if (false == stRegionDispRules.regionAvgTempDispEnable.Equals(0))
                                {
                                    writer.WriteLine("RegionID: {0}; Avg Temp :{1}; PointX :  {2}; PointY :{3}",
                                    stOutCome.regionId, 0.1 * stOutCome.avrTmp, stOutComePoint0.x, stOutComePoint0.y);

                                }

                                if (false == stRegionDispRules.regionMaxTempDispEnable.Equals(0))
                                {
                                    writer.WriteLine("RegionID: {0}; Max Temp : {1}; PointX : {2}; PointY : {3}",
                                    stOutCome.regionId, 0.1 * stOutCome.maxTmp, stOutComePoint0.x, stOutComePoint0.y);
                                }

                                if (false == stRegionDispRules.regionMinTempDispEnable.Equals(0))
                                {
                                    writer.WriteLine("RegionID: {0}; Min Temp : {1}; PointX : {2}; PointY : {3}",
                                    stOutCome.regionId, 0.1 * stOutCome.minTmp, stOutComePoint1.x, stOutComePoint1.y);
                                }
                            }
                        }

                        m_MyCamera.MV_CC_FreeImageBuffer_NET(ref stFrameInfo);
                    }
                }
            }
        }

        private void bnStartGrab_Click(object sender, EventArgs e)
        {
            if (false == m_bOpenDevice || true == m_bStartGrabbing || null == m_MyCamera)
            {
                return;
            }

            // ch:前置配置 | en:pre-operation
            int nRet = NecessaryOperBeforeGrab();
            if (MyCamera.MV_OK != nRet)
            {
                return;
            }

            displayHandle = pbDisplay.Handle;

            // ch:标志位置true | en:Set position bit true
            m_bGrabbing = true;

            m_stFrameInfo.nFrameLen = 0;//取流之前先清除帧长度
            m_stFrameInfo.enPixelType = MyCamera.MvGvspPixelType.PixelType_Gvsp_Undefined;

            m_hReceiveThread = new Thread(ReceiveThreadProcess);
            m_hReceiveThread.Start();

            // ch:开始采集 | en:Start Grabbing
            nRet = m_MyCamera.MV_CC_StartGrabbing_NET();
            if (MyCamera.MV_OK != nRet)
            {
                m_bGrabbing = false;
                m_hReceiveThread.Join();
                ShowErrorMsg("Start Grabbing Fail!", nRet);
                return;
            }

            m_bStartGrabbing = true;
            EnableControls(true);
        }

        private void bnClose_Click(object sender, EventArgs e)
        {
            // ch:取流标志位清零 | en:Reset flow flag bit
            if (m_bGrabbing == true)
            {
                m_bGrabbing = false;
                m_hReceiveThread.Join();
            }

            if (m_BufForDriver != IntPtr.Zero)
            {
                Marshal.Release(m_BufForDriver);
            }

            // ch:关闭设备 | en:Close Device
            m_MyCamera.MV_CC_CloseDevice_NET();
            m_MyCamera.MV_CC_DestroyDevice_NET();

            m_bStartGrabbing = false;
            m_bOpenDevice = false;

            // ch:控件操作 | en:Control Operation
            EnableControls(true);
        }

        private void bnStopGrab_Click(object sender, EventArgs e)
        {
            if (false == m_bOpenDevice || false == m_bStartGrabbing || null == m_MyCamera)
            {
                return;
            }

            // ch:标志位设为false | en:Set flag bit false
            m_bGrabbing = false;
            m_hReceiveThread.Join();

            // ch:停止采集 | en:Stop Grabbing
            int nRet = m_MyCamera.MV_CC_StopGrabbing_NET();
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Stop Grabbing Fail!", nRet);
            }

            m_bStartGrabbing = false;
            // ch:控件操作 | en:Control Operation
            EnableControls(true);
        }

        private void cbPixelFormat_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (false == m_bStartGrabbing)
            {
                int nRet = SetEnumIntoCombo("PixelFormat", ref cbPixelFormat);
                if (nRet != MyCamera.MV_OK)
                {
                    ShowErrorMsg("Set PixelFormat Fail!", nRet);
                }
            }
        }

        private void cbDisplaySource_SelectedIndexChanged(object sender, EventArgs e)
        {
            int nRet = SetEnumIntoCombo("OverScreenDisplayProcessor", ref cbDisplaySource);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set OverScreenDisplayProcessor Fail!", nRet);
            }
        }

        private void cbPaletteMode_SelectedIndexChanged(object sender, EventArgs e)
        {
            int nRet = SetEnumIntoCombo("PalettesMode", ref cbPaletteMode);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set PaletteMode Fail!", nRet);
            }
        }

        private void cbLegendCheck_CheckedChanged(object sender, EventArgs e)
        {
            bool bLegendCheck = cbLegendCheck.Checked;

            int nRet = m_MyCamera.MV_CC_SetBoolValue_NET("LegendDisplayEnable", bLegendCheck);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set LegendDisplayEnable Fail!", nRet);
                return;
            }

            nRet = m_MyCamera.MV_CC_SetCommandValue_NET("TempControlLoad");
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Exec TempControlLoad Fail!", nRet);
            }
        }

        private void cbExportModeCheck_CheckedChanged(object sender, EventArgs e)
        {
            bool bExportModeCheck = cbExportModeCheck.Checked;

            int nRet = m_MyCamera.MV_CC_SetBoolValue_NET("MtExpertMode", bExportModeCheck);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set ExpertMode Fail!", nRet);
                return;
            }

            nRet = m_MyCamera.MV_CC_SetCommandValue_NET("TempControlLoad");
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Exec TempControlLoad Fail!", nRet);
            }
        }

        private void cbRegionSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            int nRet = SetEnumIntoCombo("TempRegionSelector", ref cbRegionSelect);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set TempRegionSelector Fail!", nRet);
            }
        }

        private void cbMeasureRange_SelectedIndexChanged(object sender, EventArgs e)
        {
            int nRet = SetEnumIntoCombo("TempMeasurementRange", ref cbMeasureRange);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set TempMeasurementRange Fail!", nRet);
            }
        }

        private void bnSetParameter_Click(object sender, EventArgs e)
        {
            try
            {
                int.Parse(teTransmissivity.Text);
                float.Parse(teTargetDistance.Text);
                float.Parse(teEmissivity.Text);
            }
            catch
            {
                ShowErrorMsg("Please enter correct type!", 0);
                return;
            }

            int nRet = m_MyCamera.MV_CC_SetIntValueEx_NET("AtmosphericTransmissivity", int.Parse(teTransmissivity.Text));
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set AtmosphericTransmissivity Fail!", nRet);
            }

            nRet = m_MyCamera.MV_CC_SetFloatValue_NET("TargetDistance", float.Parse(teTargetDistance.Text));
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set TargetDistance Fail!", nRet);
            }

            nRet = m_MyCamera.MV_CC_SetFloatValue_NET("FullScreenEmissivity", float.Parse(teEmissivity.Text) + 0.000001F);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set FullScreenEmissivity Fail!", nRet);
            }
        }

        private void bnRegionSetting_Click(object sender, EventArgs e)
        {
            bool bExportModeCheck = cbExportModeCheck.Checked;
            InfraredDemo.RegionSettingForm = new FormRegionSetting(ref m_MyCamera, ref cbRegionSelect, ref bExportModeCheck);

            InfraredDemo.RegionSettingForm.Show();
            RegionSettingForm.Show();
        }

        private void bnWarningSetting_Click(object sender, EventArgs e)
        {
            InfraredDemo.AlarmSettingForm = new FormAlarmSetting(ref m_MyCamera, ref cbRegionSelect);
            InfraredDemo.AlarmSettingForm.Show();
            AlarmSettingForm.Show();
        }

        private void InfraredDemo_Closing(object sender, FormClosingEventArgs e)
        {
            bnClose_Click(sender, null);

            // ch: 反初始化SDK | en: Finalize SDK
            MyCamera.MV_CC_Finalize_NET();
        }
    }
}
