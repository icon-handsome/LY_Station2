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
using System.Text.RegularExpressions;
using System.Threading;
using System.IO;

using System.Drawing.Imaging;
using System.Diagnostics;
using System.Collections.ObjectModel;

namespace BasicDemoByGenTL
{
    public partial class Form1 : Form
    {
        MyCamera.MV_CC_DEVICE_INFO_LIST m_stDeviceList = new MyCamera.MV_CC_DEVICE_INFO_LIST();
        private MyCamera m_MyCamera = new MyCamera();
        bool m_bGrabbing = false;
        Thread m_hReceiveThread = null;
        UInt32 exposureNum = 0;  // 分时频闪的灯数(用户可根据相机具体的节点等方式自定义曝光个数)
        UInt32 m_nDisplayNum = 4;
        IntPtr[] displayHandles = new IntPtr[4];

        public Form1()
        {
            InitializeComponent();
            Control.CheckForIllegalCrossThreadCalls = false;

            this.Load += new EventHandler(this.Form1_Load);
            btnEnumDevice.Enabled = true;
            cmbMultiLight.Enabled = false;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // ch: 初始化 SDK | en: Initialize SDK
            MyCamera.MV_CC_Initialize_NET();
        }

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
                case MyCamera.MV_E_HANDLE: errorMsg += " Error or invalid handle "; break;
                case MyCamera.MV_E_SUPPORT: errorMsg += " Not supported function "; break;
                case MyCamera.MV_E_BUFOVER: errorMsg += " Cache is full "; break;
                case MyCamera.MV_E_CALLORDER: errorMsg += " Function calling order error "; break;
                case MyCamera.MV_E_PARAMETER: errorMsg += " Incorrect parameter "; break;
                case MyCamera.MV_E_RESOURCE: errorMsg += " Applying resource failed "; break;
                case MyCamera.MV_E_NODATA: errorMsg += " No data "; break;
                case MyCamera.MV_E_PRECONDITION: errorMsg += " Precondition error, or running environment changed "; break;
                case MyCamera.MV_E_VERSION: errorMsg += " Version mismatches "; break;
                case MyCamera.MV_E_NOENOUGH_BUF: errorMsg += " Insufficient memory "; break;
                case MyCamera.MV_E_UNKNOW: errorMsg += " Unknown error "; break;
                case MyCamera.MV_E_GC_GENERIC: errorMsg += " General error "; break;
                case MyCamera.MV_E_GC_ACCESS: errorMsg += " Node accessing condition error "; break;
                case MyCamera.MV_E_ACCESS_DENIED: errorMsg += " No permission "; break;
                case MyCamera.MV_E_BUSY: errorMsg += " Device is busy, or network disconnected "; break;
                case MyCamera.MV_E_NETER: errorMsg += " Network error "; break;
            }

            MessageBox.Show(errorMsg, "PROMPT");
        }

        private string DeleteTail(string strUserDefinedName)
        {
            strUserDefinedName = Regex.Unescape(strUserDefinedName);
            int nIndex = strUserDefinedName.IndexOf("\0");
            if (nIndex >= 0)
            {
                strUserDefinedName = strUserDefinedName.Remove(nIndex);
            }

            return strUserDefinedName;
        }

        private void btnEnumDevice_Click(object sender, EventArgs e)
        {
            DeviceListAcq();

            bnOpen.Enabled = true;
        }

        private void DeviceListAcq()
        {
            // ch:创建设备列表 | en:Create Device List
            System.GC.Collect();
            cmbDeviceList.Items.Clear();
            m_stDeviceList.nDeviceNum = 0;
            //这里枚举了所有类型，根据实际情况，选择合适的枚举类型即可
            int nRet = MyCamera.MV_CC_EnumDevices_NET(MyCamera.MV_GIGE_DEVICE | MyCamera.MV_USB_DEVICE | MyCamera.MV_GENTL_GIGE_DEVICE
                | MyCamera.MV_GENTL_CAMERALINK_DEVICE | MyCamera.MV_GENTL_CXP_DEVICE | MyCamera.MV_GENTL_XOF_DEVICE, ref m_stDeviceList);
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
                        cmbDeviceList.Items.Add("GEV: " + DeleteTail(strUserDefinedName) + " (" + gigeInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cmbDeviceList.Items.Add("GEV: " + gigeInfo.chManufacturerName + " " + gigeInfo.chModelName + " (" + gigeInfo.chSerialNumber + ")");
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
                        cmbDeviceList.Items.Add("U3V: " + DeleteTail(strUserDefinedName) + " (" + usbInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cmbDeviceList.Items.Add("U3V: " + usbInfo.chManufacturerName + " " + usbInfo.chModelName + " (" + usbInfo.chSerialNumber + ")");
                    }
                }
                else if (device.nTLayerType == MyCamera.MV_GENTL_CAMERALINK_DEVICE)
                {
                    MyCamera.MV_CML_DEVICE_INFO CMLInfo = (MyCamera.MV_CML_DEVICE_INFO)MyCamera.ByteToStruct(device.SpecialInfo.stCMLInfo, typeof(MyCamera.MV_CML_DEVICE_INFO));

                    if ((CMLInfo.chUserDefinedName.Length > 0) && (CMLInfo.chUserDefinedName[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(CMLInfo.chUserDefinedName))
                        {
                            strUserDefinedName = Encoding.UTF8.GetString(CMLInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        else
                        {
                            strUserDefinedName = Encoding.Default.GetString(CMLInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        cmbDeviceList.Items.Add("CML: " + DeleteTail(strUserDefinedName) + " (" + CMLInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cmbDeviceList.Items.Add("CML: " + CMLInfo.chManufacturerInfo + " " + CMLInfo.chModelName + " (" + CMLInfo.chSerialNumber + ")");
                    }
                }
                else if (device.nTLayerType == MyCamera.MV_GENTL_CXP_DEVICE)
                {
                    MyCamera.MV_CXP_DEVICE_INFO CXPInfo = (MyCamera.MV_CXP_DEVICE_INFO)MyCamera.ByteToStruct(device.SpecialInfo.stCXPInfo, typeof(MyCamera.MV_CXP_DEVICE_INFO));

                    if ((CXPInfo.chUserDefinedName.Length > 0) && (CXPInfo.chUserDefinedName[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(CXPInfo.chUserDefinedName))
                        {
                            strUserDefinedName = Encoding.UTF8.GetString(CXPInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        else
                        {
                            strUserDefinedName = Encoding.Default.GetString(CXPInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        cmbDeviceList.Items.Add("CXP: " + DeleteTail(strUserDefinedName) + " (" + CXPInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cmbDeviceList.Items.Add("CXP: " + CXPInfo.chManufacturerInfo + " " + CXPInfo.chModelName + " (" + CXPInfo.chSerialNumber + ")");
                    }
                }
                else if (device.nTLayerType == MyCamera.MV_GENTL_XOF_DEVICE)
                {
                    MyCamera.MV_XOF_DEVICE_INFO XOFInfo = (MyCamera.MV_XOF_DEVICE_INFO)MyCamera.ByteToStruct(device.SpecialInfo.stXoFInfo, typeof(MyCamera.MV_XOF_DEVICE_INFO));

                    if ((XOFInfo.chUserDefinedName.Length > 0) && (XOFInfo.chUserDefinedName[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(XOFInfo.chUserDefinedName))
                        {
                            strUserDefinedName = Encoding.UTF8.GetString(XOFInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        else
                        {
                            strUserDefinedName = Encoding.Default.GetString(XOFInfo.chUserDefinedName).TrimEnd('\0');
                        }
                        cmbDeviceList.Items.Add("XOF: " + DeleteTail(strUserDefinedName) + " (" + XOFInfo.chSerialNumber + ")");
                    }
                    else
                    {
                        cmbDeviceList.Items.Add("XOF: " + XOFInfo.chManufacturerInfo + " " + XOFInfo.chModelName + " (" + XOFInfo.chSerialNumber + ")");
                    }
                }
            }

            // ch:选择第一项 | en:Select the first item
            if (m_stDeviceList.nDeviceNum != 0)
            {
                cmbDeviceList.SelectedIndex = 0;
            }
        }

        private void SetCtrlWhenOpen()
        {
            btnEnumDevice.Enabled = true;
            bnOpen.Enabled = false;

            bnClose.Enabled = true;
            bnStartGrab.Enabled = true;
            bnStopGrab.Enabled = false;
            bnContinuesMode.Enabled = true;
            bnContinuesMode.Checked = true;
            bnTriggerMode.Enabled = true;
            cbSoftTrigger.Enabled = false;
            bnTriggerExec.Enabled = false;
            cmbMultiLight.Enabled = true;
        }

        private void bnOpen_Click(object sender, EventArgs e)
        {
            if (m_stDeviceList.nDeviceNum == 0 || cmbDeviceList.SelectedIndex == -1)
            {
                ShowErrorMsg("No device, please select", 0);
                return;
            }

            // ch:获取选择的设备信息 | en:Get selected device information
            MyCamera.MV_CC_DEVICE_INFO device =
                (MyCamera.MV_CC_DEVICE_INFO)Marshal.PtrToStructure(m_stDeviceList.pDeviceInfo[cmbDeviceList.SelectedIndex],
                                                              typeof(MyCamera.MV_CC_DEVICE_INFO));
            
            // ch:打开设备 | en:Open device
            if (null == m_MyCamera)
            {
                m_MyCamera = new MyCamera();
                if (null == m_MyCamera)
                {
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

            MyCamera.MVCC_ENUMVALUE stParam = new MyCamera.MVCC_ENUMVALUE();
            nRet = m_MyCamera.MV_CC_GetEnumValue_NET("MultiLightControl", ref stParam);
            if (MyCamera.MV_OK == nRet)
            {
                radioButtonFromDevice.Enabled = true;
                radioButtonFromDevice.Select();

                radioButtonFromUserInput.Enabled = false;

                cmbMultiLight.Items.Clear();
                for (int i = 0; i < stParam.nSupportedNum; i++)
                {
                    string str = "";
                    MyCamera.MVCC_ENUMENTRY stEnumEntry = new MyCamera.MVCC_ENUMENTRY();
                    stEnumEntry.nValue = stParam.nSupportValue[i];
                    m_MyCamera.MV_CC_GetEnumEntrySymbolic_NET("MultiLightControl", ref stEnumEntry);
                    if ((stEnumEntry.chSymbolic.Length > 0) && (stEnumEntry.chSymbolic[0] != '\0'))
                    {
                        if (MyCamera.IsTextUTF8(stEnumEntry.chSymbolic))
                        {
                            str = Encoding.UTF8.GetString(stEnumEntry.chSymbolic).TrimEnd('\0');
                        }
                        else
                        {
                            str = Encoding.Default.GetString(stEnumEntry.chSymbolic).TrimEnd('\0');
                        }
                        cmbMultiLight.Items.Add(DeleteTail(str));
                    }

                    // ch:显示当前选项 | en:Show current item
                    if (stParam.nCurValue == stEnumEntry.nValue)
                    {
                        cmbMultiLight.SelectedIndex = i;
                    }
                }

                exposureNum = stParam.nCurValue & 0xF;
            }
            else
            {
                radioButtonFromUserInput.Enabled = true;
                radioButtonFromUserInput.Select();
                radioButtonFromDevice.Enabled = false;

                cmbMultiLight.Items.Clear();
                string[] userInputValues = {"1", "2", "4"};
                cmbMultiLight.Items.AddRange(userInputValues);
                cmbMultiLight.SelectedIndex = 0;

                exposureNum = uint.Parse(cmbMultiLight.Text);
            }

            // ch:设置采集连续模式 | en:Set Continues Acquisition Mode
            m_MyCamera.MV_CC_SetEnumValue_NET("AcquisitionMode", (uint)MyCamera.MV_CAM_ACQUISITION_MODE.MV_ACQ_MODE_CONTINUOUS);
            m_MyCamera.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_OFF);

            // ch:控件操作 | en:Control operation
            SetCtrlWhenOpen();
        }

        private void SetCtrlWhenClose()
        {
            btnEnumDevice.Enabled = true;

            bnOpen.Enabled = true;

            bnClose.Enabled = false;
            bnStartGrab.Enabled = false;
            bnStopGrab.Enabled = false;
            bnContinuesMode.Enabled = false;
            bnTriggerMode.Enabled = false;
            cbSoftTrigger.Enabled = false;
            bnTriggerExec.Enabled = false;
            cmbMultiLight.Enabled = false;
        }

        private void bnClose_Click(object sender, EventArgs e)
        {
            // ch:取流标志位清零 | en:Reset flow flag bit
            if (m_bGrabbing == true)
            {
                m_bGrabbing = false;
                m_hReceiveThread.Join();
            }

            // ch:关闭设备 | en:Close Device
            m_MyCamera.MV_CC_CloseDevice_NET();
            m_MyCamera.MV_CC_DestroyDevice_NET();

            // ch:控件操作 | en:Control Operation
            SetCtrlWhenClose();
        }

        private void bnContinuesMode_CheckedChanged(object sender, EventArgs e)
        {
            if (bnContinuesMode.Checked)
            {
                m_MyCamera.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_OFF);
                cbSoftTrigger.Enabled = false;
                bnTriggerExec.Enabled = false;
            }
        }

        private void bnTriggerMode_CheckedChanged(object sender, EventArgs e)
        {
            // ch:打开触发模式 | en:Open Trigger Mode
            if (bnTriggerMode.Checked)
            {
                m_MyCamera.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_ON);

                // ch:触发源选择:0 - Line0; | en:Trigger source select:0 - Line0;
                //           1 - Line1;
                //           2 - Line2;
                //           3 - Line3;
                //           4 - Counter;
                //           7 - Software;
                if (cbSoftTrigger.Checked)
                {
                    m_MyCamera.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_SOFTWARE);
                    if (m_bGrabbing)
                    {
                        bnTriggerExec.Enabled = true;
                    }
                }
                else
                {
                    m_MyCamera.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_LINE0);
                }
                cbSoftTrigger.Enabled = true;
            }
        }

        private void SetCtrlWhenStartGrab()
        {
            bnStartGrab.Enabled = false;
            bnStopGrab.Enabled = true;

            if (bnTriggerMode.Checked && cbSoftTrigger.Checked)
            {
                bnTriggerExec.Enabled = true;
            }
        }

        public void ReceiveThreadProcess()
        {
            int nRet = MyCamera.MV_OK;
            MyCamera.MV_FRAME_OUT stImageInfo = new MyCamera.MV_FRAME_OUT();
            MyCamera.MV_CC_HB_DECODE_PARAM stDecodeParam = new MyCamera.MV_CC_HB_DECODE_PARAM();
            MyCamera.MV_DISPLAY_FRAME_INFO stDisplayInfo = new MyCamera.MV_DISPLAY_FRAME_INFO();
            MyCamera.MV_RECONSTRUCT_IMAGE_PARAM stImgReconstructionParam = new MyCamera.MV_RECONSTRUCT_IMAGE_PARAM();
            stImgReconstructionParam.stDstBufList = new MyCamera.MV_OUTPUT_IMAGE_INFO[MyCamera.MV_MAX_SPLIT_NUM];
            for (int i = 0; i < MyCamera.MV_MAX_SPLIT_NUM; i++)
            {
                stImgReconstructionParam.stDstBufList[i] = new MyCamera.MV_OUTPUT_IMAGE_INFO();
            }
            
            UInt32   nImageBufferSize = 0;

            IntPtr[] pImageBufferList = new IntPtr[m_nDisplayNum];
    
            MyCamera.MVCC_INTVALUE_EX stPayloadSize = new MyCamera.MVCC_INTVALUE_EX();
            m_MyCamera.MV_CC_GetIntValueEx_NET("PayloadSize", ref stPayloadSize);

            while (m_bGrabbing)
            {
                nRet = m_MyCamera.MV_CC_GetImageBuffer_NET(ref stImageInfo, 1000);
                if (nRet == MyCamera.MV_OK)
                {
                    // ch:HB格式需要先解码 | en:Decode image when HB mode is enable
                    if (stImageInfo.stFrameInfo.enPixelType.ToString().Contains("HB"))
                    {
                        stDecodeParam.pSrcBuf = stImageInfo.pBufAddr;
                        stDecodeParam.nSrcLen = stImageInfo.stFrameInfo.nFrameLen;

                        if (stDecodeParam.nDstBufSize < stPayloadSize.nCurValue)
                        {
                            if (stDecodeParam.pDstBuf != IntPtr.Zero)
                            {
                                Marshal.FreeHGlobal(stDecodeParam.pDstBuf);
                                stDecodeParam.pDstBuf = IntPtr.Zero;
                            }

                            stDecodeParam.pDstBuf = Marshal.AllocHGlobal((int)stPayloadSize.nCurValue);
                            if (stDecodeParam.pDstBuf == IntPtr.Zero)
                            {
                                m_MyCamera.MV_CC_FreeImageBuffer_NET(ref stImageInfo);
                                continue;
                            }
                            stDecodeParam.nDstBufSize = (uint)stPayloadSize.nCurValue;
                        }

                        nRet = m_MyCamera.MV_CC_HB_Decode_NET(ref stDecodeParam);
                        if (MyCamera.MV_OK != nRet)
                        {
                            m_MyCamera.MV_CC_FreeImageBuffer_NET(ref stImageInfo);
                            continue;
                        }
                    }

                    // ch:图像重组 | en:Reconstruct Image
                    if (exposureNum > 1)      // 多灯
                    {
                        if (stImageInfo.stFrameInfo.enPixelType.ToString().Contains("HB"))
                        {
                            stImgReconstructionParam.nWidth = stDecodeParam.nWidth;
                            stImgReconstructionParam.nHeight = stDecodeParam.nHeight;
                            stImgReconstructionParam.enPixelType = stDecodeParam.enDstPixelType;
                            stImgReconstructionParam.pSrcData = stDecodeParam.pDstBuf;
                            stImgReconstructionParam.nSrcDataLen = stDecodeParam.nDstBufLen;
                        }
                        else
                        {
                            stImgReconstructionParam.nWidth = stImageInfo.stFrameInfo.nWidth;
                            stImgReconstructionParam.nHeight = stImageInfo.stFrameInfo.nHeight;
                            stImgReconstructionParam.enPixelType = stImageInfo.stFrameInfo.enPixelType;
                            stImgReconstructionParam.pSrcData = stImageInfo.pBufAddr;
                            stImgReconstructionParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
                        }

                        stImgReconstructionParam.nExposureNum = exposureNum;
                        stImgReconstructionParam.enReconstructMethod = MyCamera.MV_IMAGE_RECONSTRUCTION_METHOD.MV_SPLIT_BY_LINE;

                        if (stImgReconstructionParam.nSrcDataLen / exposureNum > nImageBufferSize)
                        {
                            for (int i = 0; i < m_nDisplayNum; i++)
                            {
                                if (pImageBufferList[i] != IntPtr.Zero)
                                {
                                    Marshal.FreeHGlobal(pImageBufferList[i]);
                                    pImageBufferList[i] = IntPtr.Zero;
                                }
                                pImageBufferList[i] = Marshal.AllocHGlobal((Int32)(stImgReconstructionParam.nSrcDataLen / exposureNum)); ;
                                if (pImageBufferList[i] != IntPtr.Zero)
                                {
                                    stImgReconstructionParam.stDstBufList[i].pBuf = pImageBufferList[i];
                                    stImgReconstructionParam.stDstBufList[i].nBufSize = stImgReconstructionParam.nSrcDataLen / exposureNum;
                                }
                                else
                                {
                                    return;
                                }
                            }

                            nImageBufferSize = stImgReconstructionParam.nSrcDataLen / exposureNum;
                        }
                        nRet = m_MyCamera.MV_CC_ReconstructImage_NET(ref stImgReconstructionParam);
                        if (nRet != MyCamera.MV_OK)
                        {
                            m_MyCamera.MV_CC_FreeImageBuffer_NET(ref stImageInfo);
                            continue;
                        }

                        for (int i = 0; i < exposureNum; i++)
                        {
                            stDisplayInfo.hWnd = displayHandles[i];
                            stDisplayInfo.pData = stImgReconstructionParam.stDstBufList[i].pBuf;
                            stDisplayInfo.nDataLen = stImgReconstructionParam.stDstBufList[i].nBufLen;
                            stDisplayInfo.nWidth = (UInt16)stImgReconstructionParam.stDstBufList[i].nWidth;
                            stDisplayInfo.nHeight = (UInt16)stImgReconstructionParam.stDstBufList[i].nHeight;
                            stDisplayInfo.enPixelType = stImgReconstructionParam.stDstBufList[i].enPixelType;
                            m_MyCamera.MV_CC_DisplayOneFrame_NET(ref stDisplayInfo);
                        }
                    }
                    else
                    {
                        stDisplayInfo.hWnd = displayHandles[0];
                        if (stImageInfo.stFrameInfo.enPixelType.ToString().Contains("HB"))
                        {
                            stDisplayInfo.nWidth = (ushort)stDecodeParam.nWidth;
                            stDisplayInfo.nHeight = (ushort)stDecodeParam.nHeight;
                            stDisplayInfo.enPixelType = stDecodeParam.enDstPixelType;
                            stDisplayInfo.pData = stDecodeParam.pDstBuf;
                            stDisplayInfo.nDataLen = stDecodeParam.nDstBufLen;
                        }
                        else
                        {
                            stDisplayInfo.pData = stImageInfo.pBufAddr;
                            stDisplayInfo.nDataLen = stImageInfo.stFrameInfo.nFrameLen;
                            stDisplayInfo.nWidth = stImageInfo.stFrameInfo.nWidth;
                            stDisplayInfo.nHeight = stImageInfo.stFrameInfo.nHeight;
                            stDisplayInfo.enPixelType = stImageInfo.stFrameInfo.enPixelType;
                        }

                        m_MyCamera.MV_CC_DisplayOneFrame_NET(ref stDisplayInfo);
                    }

                    m_MyCamera.MV_CC_FreeImageBuffer_NET(ref stImageInfo);
                }
                else
                {
                    if (bnTriggerMode.Checked)
                    {
                        Thread.Sleep(5);
                    }
                }
            }

            for (int i = 0; i < m_nDisplayNum; i++)
            {
                if (pImageBufferList[i] != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(pImageBufferList[i]);
                    pImageBufferList[i] = IntPtr.Zero;
                }
            }

            if (stDecodeParam.pDstBuf != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(stDecodeParam.pDstBuf);
                stDecodeParam.pDstBuf = IntPtr.Zero;
            }
        }

        private void bnStartGrab_Click(object sender, EventArgs e)
        {
            pictureBox1.Refresh();
            pictureBox2.Refresh();
            pictureBox3.Refresh();
            pictureBox4.Refresh();

            displayHandles[0] = pictureBox1.Handle;
            displayHandles[1] = pictureBox2.Handle;
            displayHandles[2] = pictureBox3.Handle;
            displayHandles[3] = pictureBox4.Handle;

            cmbMultiLight.Enabled = false;
            // ch:标志位置位true | en:Set position bit true
            m_bGrabbing = true;

            m_hReceiveThread = new Thread(ReceiveThreadProcess);
            m_hReceiveThread.Start();

            // ch:开始采集 | en:Start Grabbing
            int nRet = m_MyCamera.MV_CC_StartGrabbing_NET();
            if (MyCamera.MV_OK != nRet)
            {
                m_bGrabbing = false;
                m_hReceiveThread.Join();
                ShowErrorMsg("Start Grabbing Fail!", nRet);
                return;
            }

            // ch:控件操作 | en:Control Operation
            SetCtrlWhenStartGrab();
        }

        private void cbSoftTrigger_CheckedChanged(object sender, EventArgs e)
        {
            if (cbSoftTrigger.Checked)
            {
                // ch:触发源设为软触发 | en:Set trigger source as Software
                m_MyCamera.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_SOFTWARE);
                if (m_bGrabbing)
                {
                    bnTriggerExec.Enabled = true;
                }
            }
            else
            {
                m_MyCamera.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_LINE0);
                bnTriggerExec.Enabled = false;
            }
        }

        private void bnTriggerExec_Click(object sender, EventArgs e)
        {
            // ch:触发命令 | en:Trigger command
            int nRet = m_MyCamera.MV_CC_SetCommandValue_NET("TriggerSoftware");
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Trigger Software Fail!", nRet);
            }
        }

        private void SetCtrlWhenStopGrab()
        {
            bnStartGrab.Enabled = true;
            bnStopGrab.Enabled = false;

            bnTriggerExec.Enabled = false;
        }

        private void bnStopGrab_Click(object sender, EventArgs e)
        {
            cmbMultiLight.Enabled = true;
            // ch:标志位设为false | en:Set flag bit false
            m_bGrabbing = false;
            m_hReceiveThread.Join();

            // ch:停止采集 | en:Stop Grabbing
            int nRet = m_MyCamera.MV_CC_StopGrabbing_NET();
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Stop Grabbing Fail!", nRet);
            }

            // ch:控件操作 | en:Control Operation
            SetCtrlWhenStopGrab();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            bnClose_Click(sender, e);

            // ch: 反初始化SDK | en: Finalize SDK
            MyCamera.MV_CC_Finalize_NET();
        }

        private void cmbMultiLight_SelectionChangeCommitted(object sender, EventArgs e)
        {
            if (radioButtonFromDevice.Enabled)
            {
                int nRet = m_MyCamera.MV_CC_SetEnumValueByString_NET("MultiLightControl", cmbMultiLight.Text);
                if (MyCamera.MV_OK != nRet)
                {
                    ShowErrorMsg("Set MultiLightControl failed", nRet);
                    return;
                }

                MyCamera.MVCC_ENUMVALUE stEnumValue = new MyCamera.MVCC_ENUMVALUE();
                nRet = m_MyCamera.MV_CC_GetEnumValue_NET("MultiLightControl", ref stEnumValue);
                if (MyCamera.MV_OK == nRet)
                {
                    exposureNum = stEnumValue.nCurValue & 0xF;
                }
            }
            else
            {
                exposureNum = uint.Parse(cmbMultiLight.Text);
            }

        }
    }
}