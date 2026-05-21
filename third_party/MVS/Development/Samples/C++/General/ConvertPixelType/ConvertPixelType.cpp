#include <stdio.h>
#include <Windows.h>
#include <conio.h>
#include "MvCameraControl.h"

// ch:等待按键输入 | en:Wait for key press
void WaitForKeyPress(void)
{
    while(!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        printf("Model Name %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

bool IsColor(MvGvspPixelType enType)
{
    switch(enType)
    {
    case PixelType_Gvsp_BGR8_Packed:
    case PixelType_Gvsp_YUV422_Packed:
    case PixelType_Gvsp_YUV422_YUYV_Packed:
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerGB10_Packed:
    case PixelType_Gvsp_BayerBG10:
    case PixelType_Gvsp_BayerBG10_Packed:
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerRG10_Packed:
    case PixelType_Gvsp_BayerGR10:
    case PixelType_Gvsp_BayerGR10_Packed:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerGB12_Packed:
    case PixelType_Gvsp_BayerBG12:
    case PixelType_Gvsp_BayerBG12_Packed:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerRG12_Packed:
    case PixelType_Gvsp_BayerGR12:
    case PixelType_Gvsp_BayerGR12_Packed:
	case PixelType_Gvsp_BayerRBGG8:
	case PixelType_Gvsp_BayerGR16:
	case PixelType_Gvsp_BayerRG16:
	case PixelType_Gvsp_BayerGB16:
	case PixelType_Gvsp_BayerBG16:
        return true;
    default:
        return false;
    }
}

bool IsMono(MvGvspPixelType enType)
{
    switch(enType)
    {
    case PixelType_Gvsp_Mono10:
    case PixelType_Gvsp_Mono10_Packed:
    case PixelType_Gvsp_Mono12:
    case PixelType_Gvsp_Mono12_Packed:
	case PixelType_Gvsp_Mono14:
	case PixelType_Gvsp_Mono16:
        return true;
    default:
        return false;
    }
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
    unsigned char *pConvertData = NULL;
    unsigned int nConvertDataSize = 0;

    do 
    {
		// ch:初始化SDK | en:Initialize SDK
		nRet = MV_CC_Initialize();
		if (MV_OK != nRet)
		{
			printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
			break;
		}

        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum-1);
        unsigned int nIndex = 0;
        scanf_s("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Input error!\n");
            break;
        }

        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValueEx(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        MV_FRAME_OUT stImageInfo = {0};

        nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n", 
                stImageInfo.stFrameInfo.nExtendWidth, stImageInfo.stFrameInfo.nExtendHeight, stImageInfo.stFrameInfo.nFrameNum);

            MvGvspPixelType enDstPixelType = PixelType_Gvsp_Undefined;
            unsigned int nChannelNum = 0;
            char chFileName[MAX_PATH] = {0};
            // ch:如果是彩色则转成RGB8 | en:if pixel type is color, convert it to RGB8
            if (IsColor(stImageInfo.stFrameInfo.enPixelType))
            {
                nChannelNum = 3;
                enDstPixelType = PixelType_Gvsp_RGB8_Packed;
                sprintf(chFileName, "AfterConvertRGB.raw");
            }
            // ch:如果是黑白则转换成Mono8 | en:if pixel type is mono, convert it to mono8
            else if (IsMono(stImageInfo.stFrameInfo.enPixelType))
            {
                nChannelNum = 1;
                enDstPixelType = PixelType_Gvsp_Mono8;
                sprintf(chFileName, "AfterConvertMono8.raw");
            }
            else
            {
                printf("Don't need to convert!\n");
            }
            
            if (enDstPixelType != PixelType_Gvsp_Undefined)
            {
				// ch:设置插值算法为均衡 | en:set interpolation algorithm type, 0-Fast 1-Equilibrium 2-Optimal 3-Optimal plus
				nRet = MV_CC_SetBayerCvtQuality(handle, 1);
				if (MV_OK != nRet)
				{
					printf("set Bayer convert quality fail! nRet [0x%x]\n", nRet);
					break;
				}

                pConvertData = (unsigned char*)malloc(stImageInfo.stFrameInfo.nExtendWidth * stImageInfo.stFrameInfo.nExtendHeight * nChannelNum);
                if (NULL == pConvertData)
                {
                    printf("malloc pConvertData fail!\n");
                    break;
                }
                nConvertDataSize = stImageInfo.stFrameInfo.nExtendWidth * stImageInfo.stFrameInfo.nExtendHeight * nChannelNum;

                // ch:像素格式转换 | en:Convert pixel format 
                MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {0};

                stConvertParam.nWidth = stImageInfo.stFrameInfo.nExtendWidth;                 //ch:图像宽 | en:image width
                stConvertParam.nHeight = stImageInfo.stFrameInfo.nExtendHeight;               //ch:图像高 | en:image height
                stConvertParam.pSrcData = stImageInfo.pBufAddr;                         //ch:输入数据缓存 | en:input data buffer
				stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;         //ch:输入数据大小 | en:input data size
                stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format
                stConvertParam.enDstPixelType = enDstPixelType;                         //ch:输出像素格式 | en:output pixel format
                stConvertParam.pDstBuffer = pConvertData;                               //ch:输出数据缓存 | en:output data buffer
                stConvertParam.nDstBufferSize = nConvertDataSize;                       //ch:输出缓存大小 | en:output buffer size
                nRet = MV_CC_ConvertPixelTypeEx(handle, &stConvertParam);
                if (MV_OK != nRet)
                {
                    printf("Convert Pixel Type fail! nRet [0x%x]\n", nRet);
                    break;
                }

                FILE* fp = NULL;
                errno_t err = fopen_s(&fp, chFileName, "wb");
                if (0 != err || NULL == fp)
                {
                    printf("Open file failed\n");
                    break;
                }
                fwrite(stConvertParam.pDstBuffer, 1, stConvertParam.nDstLen, fp);
                fclose(fp);
                printf("Convert pixeltype succeed\n");
            }
            MV_CC_FreeImageBuffer(handle, &stImageInfo);
        }
        else
        {
            printf("Get Image fail! nRet [0x%x]\n", nRet);
        }

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:关闭设备 | en:Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Close Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | en:Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
		handle = NULL;
    } while (0);

    if (pConvertData)
    {
        free(pConvertData);
        pConvertData = NULL;
    }

	if (handle != NULL)
	{
		MV_CC_DestroyHandle(handle);
		handle = NULL;
	}
   

	// ch:反初始化SDK | en:Finalize SDK
	MV_CC_Finalize();

    printf("Press a key to exit.\n");
    WaitForKeyPress();

    return 0;
}
