/************************************************************************/
/*      工业相机SDK使用GPUDirect For Video参考样例代码                */
/************************************************************************/
#include "stdafx.h"
#include <dvpapi_cuda.h>
#include "MvCameraControl.h"
#include <conio.h>
#include <stdio.h>
#include <Windows.h>
#include <process.h>
#include <list>
#include <iostream>


#define BUFFER_NUMBER       200        // ch:申请的缓存个数 | en:Number of requested buffer
uint64_t  g_nPayloadSize = 0;
unsigned int g_nWidth    = 0;
unsigned int g_nHeight   = 0;
uint64_t g_nListNum = 0;


bool g_bExit = false;

// synchronization
typedef CRITICAL_SECTION Lock;

inline bool lock_create(Lock* lock) { InitializeCriticalSection(lock); return true; }
inline void lock_destroy(Lock* lock) { DeleteCriticalSection(lock); }
inline void lock_acquire(Lock* lock) { EnterCriticalSection(lock); }
inline void lock_release(Lock* lock) { LeaveCriticalSection(lock); }


Lock g_csLock;

typedef struct _DVP_SYNCINFO_ 
{
    volatile uint32_t* pnSem;       ///< 信号量指针
    volatile uint32_t* pnSemOrg;    ///< 信号量源指针
    volatile uint32_t nReleaseValue;///< 释放数据对应信号量值
    volatile uint32_t nAcquireValue;///< 获取数据对应信号量值
    DVPSyncObjectHandle hSyncObj;   ///< DVP 同步对象句柄
} DVP_SYNCINFO;///< DVP 同步信息

typedef struct _THREAD_INFO_
{
    void*           hCamHandle;                  ///< 相机句柄
    void*           pBuffer[BUFFER_NUMBER];      ///< 相机取流时输入缓存
    DVPBufferHandle hSysMemHandle[BUFFER_NUMBER];///< DVP系统内存句柄
    DVP_SYNCINFO    stSysMemSync[BUFFER_NUMBER]; ///< 系统内存同步信息

    DVPBufferHandle hGpuObjectHandle;            ///< GPU对象句柄
    DVP_SYNCINFO    stGpuSync;                   ///< GPU同步信息

    CUdeviceptr     cudaBuffer;                  ///< cuda设备缓存
    CUcontext       cuCtx;                       ///< cuda上下文

}THREAD_INFO;///< 线程输入参数信息y

double My_Milliseconds()
{
#ifdef WIN32
    LARGE_INTEGER ticks;
    QueryPerformanceCounter(&ticks);
    LARGE_INTEGER resolution;
    QueryPerformanceFrequency(&resolution);
    double dticks = (double)ticks.QuadPart;
    double dresolution = (double)resolution.QuadPart;
    return dticks / dresolution * 1000.0;
#else
    return 0.0;
#endif
}


// ch:等待按键输入 | en:Wait for key press
void WaitForKeyPress(void)
{
    while (!_kbhit())
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
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
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
        printf("Model Name: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
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

int FindBufIndex(THREAD_INFO*pstInfo, unsigned char* pBuf)
{
    if (NULL == pstInfo)
    {
        return -1;
    }

    for (unsigned int nIndex = 0; nIndex < BUFFER_NUMBER; nIndex++)
    {
        if (pstInfo->pBuffer[nIndex] == pBuf)
        {
            return nIndex;
        }
    }

    return -1;

}

int RunSysmemToGPU(DVPBufferHandle dvpSysHandle, DVP_SYNCINFO & stSysSync, DVPBufferHandle dvpGpuHandle, DVP_SYNCINFO & stGpuSync, unsigned int nDataLen)
{
    int nStatus = DVP_STATUS_OK;

    nStatus = dvpBegin();
    if (DVP_STATUS_OK != nStatus)
    {
        printf("dvpBegin fail, nStatus[%d]\n", nStatus);
        return nStatus;
    }

    do
    {

        stSysSync.nAcquireValue++;
        stGpuSync.nReleaseValue++;

        //update the release value
        stSysSync.nReleaseValue++;
        //update the semaphore when each chunk is DMAed
        *stSysSync.pnSem = stSysSync.nReleaseValue;

#if 0
        nStatus = dvpMemcpy2D(dvpSysHandle,
            stSysSync.hSyncObj,
            stSysSync.nAcquireValue,
            DVP_TIMEOUT_IGNORED,
            dvpGpuHandle,
            stGpuSync.hSyncObj,
            stGpuSync.nReleaseValue,
            0,
            0,
            g_nHeight,
            g_nWidth);
#else

        nStatus = dvpMemcpy(dvpSysHandle,
            stSysSync.hSyncObj,
            stSysSync.nAcquireValue,
            DVP_TIMEOUT_IGNORED,
            dvpGpuHandle,
            stGpuSync.hSyncObj,
            stGpuSync.nReleaseValue,
            0,
            0,
            nDataLen);
#endif
        if (DVP_STATUS_OK != nStatus)
        {
            printf("dvpMemcpy fail, nStatus[%d]\n", nStatus);
        }

    } while (false);

    nStatus = dvpEnd();
    if (DVP_STATUS_OK != nStatus)
    {
        printf("dvpEnd fail, nStatus[%d]\n", nStatus);
    }

    return nStatus;

}

std::list<MV_FRAME_OUT> listImgBuf;

///< 取流线程
static  unsigned int __stdcall WorkThread(void* pUser)
{
    if (NULL == pUser)
    {
        return 0;
    }

    int nRet = MV_OK;
    MV_FRAME_OUT stOutFrame = { 0 };

    THREAD_INFO* pstThreadInfo = (THREAD_INFO*)pUser;
    void* hCam = pstThreadInfo->hCamHandle;

    double dGetFrameBegin = 0.0;
	double dGetFrameEnd = 0.0;
	double dGetFrameTotal = 0.0;
	double dMaxGetFrameTime = 0.0;

	uint64_t nRecvNum = 0;

    nRet = MV_CC_StartGrabbing(hCam);
    if (MV_OK != nRet)
    {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    while (true)
    {
		dGetFrameBegin = My_Milliseconds();

        nRet = MV_CC_GetImageBuffer(hCam, &stOutFrame, 100);

		dGetFrameEnd = My_Milliseconds();

        if (nRet == MV_OK)
        {
			dGetFrameTotal += (dGetFrameEnd - dGetFrameBegin);
			if (dMaxGetFrameTime < (dGetFrameEnd - dGetFrameBegin))
			{
				dMaxGetFrameTime = (dGetFrameEnd - dGetFrameBegin);
			}

			nRecvNum++;

            lock_acquire(&g_csLock);

            listImgBuf.push_back(stOutFrame);

            g_nListNum++;

            lock_release(&g_csLock);

            if (g_nPayloadSize != stOutFrame.stFrameInfo.nFrameLen)
            {
                printf("FrameLen %d PayloadSize %I64d\n", stOutFrame.stFrameInfo.nFrameLen, g_nPayloadSize);
            }
 
			if (0 == nRecvNum % 65535)
			{
				printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d]\n",
					stOutFrame.stFrameInfo.nExtendWidth, stOutFrame.stFrameInfo.nExtendHeight, stOutFrame.stFrameInfo.nFrameNum);
			}
        }

        if (g_bExit)
        {
            break;
        }
    }

    nRet = MV_CC_StopGrabbing(hCam);

    printf("Stop grabbing success!\n");

	if (dGetFrameTotal > 0.0)
    {
		printf("MV_CC_GetImageBuffer MaxGetFrametime %fms, avg %fms\r\n", dMaxGetFrameTime, dGetFrameTotal / nRecvNum);
    }


    printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d], RecvNum[%I64d] \n",
		stOutFrame.stFrameInfo.nExtendWidth, stOutFrame.stFrameInfo.nExtendHeight, stOutFrame.stFrameInfo.nFrameNum, nRecvNum);

    return 0;
}

///< GPU数据处理线程
static  unsigned int __stdcall GpuThread(void* pUser)
{
    if (NULL == pUser)
    {
        return 0;
    }

    THREAD_INFO* pstThreadInfo = (THREAD_INFO*)pUser;
    void* hCam = pstThreadInfo->hCamHandle;

    int nRet = MV_OK;
    MV_FRAME_OUT stOutFrame = { 0 };
    uint64_t nFrameCount    = 0;
	uint64_t nRunCount      = 0;
	uint64_t nMaxListCount  = 0;

    double dGpuBegin   = 0.0;
    double dGpuEnd     = 0.0;
    double dGpuTotal   = 0.0;
	double dMaxGpuTime = 0.0;

    while (true)
    {
		if (g_bExit)
		{
			break;
		}

        if (listImgBuf.empty())
        {
            Sleep(0);
            continue;
        }

		if (nMaxListCount < g_nListNum)
        {
			nMaxListCount = g_nListNum;
        }

        nFrameCount++;

        lock_acquire(&g_csLock);

        stOutFrame = listImgBuf.front();
        listImgBuf.pop_front();

        g_nListNum--;

        lock_release(&g_csLock);

        int nBufIndex = FindBufIndex(pstThreadInfo, stOutFrame.pBufAddr);

        if ((nBufIndex >= 0) && (nBufIndex < BUFFER_NUMBER))
        {
            dGpuBegin = My_Milliseconds();

            nRet = RunSysmemToGPU(pstThreadInfo->hSysMemHandle[nBufIndex], 
				                  pstThreadInfo->stSysMemSync[nBufIndex],
                                  pstThreadInfo->hGpuObjectHandle, 
								  pstThreadInfo->stGpuSync,
                                  stOutFrame.stFrameInfo.nFrameLen);

            dGpuEnd = My_Milliseconds();

            dGpuTotal += (dGpuEnd - dGpuBegin);

            nRunCount++;

            if (g_nPayloadSize != stOutFrame.stFrameInfo.nFrameLen)
            {
                printf("GPU FrameLen %d PayloadSize %I64d\n", stOutFrame.stFrameInfo.nFrameLen, g_nPayloadSize);
            }

			if (dMaxGpuTime < (dGpuEnd - dGpuBegin))
            {
				dMaxGpuTime = (dGpuEnd - dGpuBegin);
            }
        }
        else
        {
            printf("Buf Not Found \n");
        }

        nRet = MV_CC_FreeImageBuffer(hCam, &stOutFrame);
        if (nRet != MV_OK)
        {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }


    printf("g_nListNum %I64d \r\n", g_nListNum);

    printf("GPUDirect: Width[%d], Height[%d], FrameNum[%d], FrameLen[%I64d],receiveNum[%I64d]\n",
        stOutFrame.stFrameInfo.nExtendWidth, stOutFrame.stFrameInfo.nExtendHeight, stOutFrame.stFrameInfo.nFrameNum, stOutFrame.stFrameInfo.nFrameLenEx, nFrameCount);

    if (dGpuTotal > 0.0)
    {
		printf("RunSysmemToGPU Maxtime %fms nMaxListCount %I64d \r\n", dMaxGpuTime, nMaxListCount);
        printf("Frame:%I64d RunSysmemToGPU cost %fms avg %fms BandWidth %fGB/s\r\n ", nRunCount, dGpuEnd - dGpuBegin, dGpuTotal / nRunCount, stOutFrame.stFrameInfo.nFrameLenEx * 1000.0 / (dGpuTotal / nRunCount) / (1024 * 1024 * 1024));
    }

    return 0;
}

uint32_t g_nSemaphoreAddrAlignment = 0;
uint32_t g_nSemaphoreAllocSize = 0;
uint32_t g_nBufferAddrAlignment = 0;

int InitCuda(CUcontext* pcuCtx)
{
    int res = CUDA_SUCCESS;
    res = cuInit(0);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }

    int nDeviceCount = 0;
    CUdevice cuDev = 0;
    res = cuDeviceGetCount(&nDeviceCount);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }

    printf("Cuda DeviceCount %d \r\n", nDeviceCount);

    res = cuDeviceGet(&cuDev, 0);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }

    printf("Cuda Device %d \r\n", cuDev);

    res = cuCtxCreate(pcuCtx, 0, 0);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }

    res = cuCtxSetCurrent(*pcuCtx);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }


    res = dvpInitCUDAContext(DVP_DEVICE_FLAGS_SHARE_APP_CONTEXT);
    if (CUDA_SUCCESS != res)
    {
        return res;
    }


	//uint32_t nBufferAddrAlignment = 0;
	uint32_t nBufferGPUStrideAlignment = 0;
	uint32_t nSemaphorePayloadOffset = 0;
	uint32_t nSemaphorePayloadSize = 0;

	res = dvpGetRequiredConstantsCUDACtx(&g_nBufferAddrAlignment,
		&nBufferGPUStrideAlignment,
        &g_nSemaphoreAddrAlignment,
		&g_nSemaphoreAllocSize,
		&nSemaphorePayloadOffset,
		&nSemaphorePayloadSize);
	if (0 == g_nBufferAddrAlignment)
    {
        printf("Not Support GPUDirect For Video \r\n");
        return -1;
    }


    printf("g_nBufferAddrAlignment %d  nBufferGPUStrideAlignment %d \r\n", g_nBufferAddrAlignment, nBufferGPUStrideAlignment);


    return res;
}

void DeinitCuda(CUcontext cuCtx)
{
     dvpCloseCUDAContext();

     cuCtxDestroy(cuCtx);
}


void SetupSyncObject(DVP_SYNCINFO* si)
{
    DVPSyncObjectDesc syncObjectDesc;
	si->pnSemOrg = (uint32_t*)malloc(g_nSemaphoreAllocSize + g_nSemaphoreAddrAlignment - 1);
    if (NULL == si->pnSemOrg)
    {
        return;
    }

    // Correct alignment
    uint64_t val = (uint64_t)si->pnSemOrg;
	val += g_nSemaphoreAddrAlignment - 1;
	val &= ~((uint64_t)g_nSemaphoreAddrAlignment - 1);
    si->pnSem = (uint32_t*)val;

    // Initialise members
    *si->pnSem = 0;
    si->nReleaseValue = 0;
    si->nAcquireValue = 0;
    syncObjectDesc.sem = (uint32_t*)si->pnSem;
    syncObjectDesc.externalClientWaitFunc = NULL;
    syncObjectDesc.flags = 0; // DVP_SYNC_OBJECT_FLAGS_USE_EVENTS;

    DVPStatus nStatus = (dvpImportSyncObject(&syncObjectDesc, &si->hSyncObj));
    if (DVP_STATUS_OK != nStatus)
    {
        printf("dvpImportSyncObject fail %d", nStatus);
    }
}



int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
    CUcontext cuCtx = NULL;

    THREAD_INFO stThreadInfo = {0};

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

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
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
                nRet = MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // ch:获取流通道图像大小 | en:Get payload size of stream
        uint64_t  nPayloadSize = 0;
        unsigned int nAlign = 0;
        nRet = MV_CC_GetPayloadSize(handle, &nPayloadSize, &nAlign);
        if (MV_OK != nRet)
        {
            printf("Get payload size failed! %#x\n", nRet);
            break;
        }
		g_nPayloadSize = nPayloadSize;

        MVCC_INTVALUE_EX stIntValue;
        nRet = MV_CC_GetIntValueEx(handle, "Width", &stIntValue);
		if (MV_OK != nRet)
		{
			printf("Get Width failed! %#x\n", nRet);
			break;
		}
        g_nWidth = stIntValue.nCurValue;

        nRet = MV_CC_GetIntValueEx(handle, "Height", &stIntValue);
		if (MV_OK != nRet)
		{
			printf("Get Height failed! %#x\n", nRet);
			break;
		}
        g_nHeight = stIntValue.nCurValue;

        printf("PayloadSize %I64d Align %d Width %d Height %d\r\n", nPayloadSize, nAlign, g_nWidth, g_nHeight);

        size_t  dwMin = 0;
		size_t  dwMax = 0;
        HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_SET_QUOTA, FALSE, GetCurrentProcessId());
		if (nullptr != hProcess)
		{
			// Retrieve the working set size of the process.
			if (!dwMin && !GetProcessWorkingSetSize(hProcess, &dwMin, &dwMax))
			{
				printf("GetProcessWorkingSetSize failed (%d)\n",
					GetLastError());
			}

			printf("GetProcessWorkingSetSize dwMin %I64d dwMax %I64d\r\n", dwMin, dwMax);

			BOOL bSuc = SetProcessWorkingSetSize(hProcess,
				                                 nPayloadSize * BUFFER_NUMBER + dwMin,
												 nPayloadSize * BUFFER_NUMBER + (dwMax - dwMin));

			if (!bSuc) 
			{
				printf("SetProcessWorkingSetSize failed (%d)\n",GetLastError());
			}

			CloseHandle(hProcess);
		}
		else
        {
            printf("OpenProcess failed (%d)\n", GetLastError());
        }


        int res = InitCuda(&cuCtx);
        if (CUDA_SUCCESS != res)
        {
            printf("InitCuda err %d \r\n", res);
            return res;
        }

        DVPSysmemBufferDesc sysMemBuffersDesc;

        for (unsigned int i = 0; i < BUFFER_NUMBER; i++)
        {
			sysMemBuffersDesc.type   = DVP_UNSIGNED_BYTE;
            sysMemBuffersDesc.format = DVP_BUFFER;
            sysMemBuffersDesc.size   = nPayloadSize;

            // ch:分配图像缓存 | en:Allocate image memory
            stThreadInfo.pBuffer[i] = VirtualAlloc(NULL, sysMemBuffersDesc.size, MEM_COMMIT | MEM_RESERVE | MEM_WRITE_WATCH, PAGE_READWRITE);
			if (NULL == stThreadInfo.pBuffer[i])
			{
				nRet = GetLastError();
				printf("%d VirtualAlloc size %d failed 0x%x\r\n", i, sysMemBuffersDesc.size, nRet);
				break;
			}

            // Pin the memory
            if (!VirtualLock(stThreadInfo.pBuffer[i], sysMemBuffersDesc.size))
            {
				nRet = GetLastError();
				printf("%d VirtualLock size %d failed 0x%x\r\n", i, sysMemBuffersDesc.size, nRet);
                break;
            }

            sysMemBuffersDesc.bufAddr = stThreadInfo.pBuffer[i];

            nRet = dvpCreateBuffer(&sysMemBuffersDesc, &stThreadInfo.hSysMemHandle[i]);
            if (DVP_STATUS_OK != nRet)
            {
                printf("%d dvpCreateBuffer buffer failed! %#x", i, nRet);
                break;
            }

            nRet = dvpBindToCUDACtx(stThreadInfo.hSysMemHandle[i]);
            if (DVP_STATUS_OK != nRet)
            {
                printf("%d dvpBindToCUDACtx buffer failed! %#x", i, nRet);
                break;
            }

            SetupSyncObject(&stThreadInfo.stSysMemSync[i]);

            // ch:向SDK注册缓存 | en:Announce buffer for SDK
            nRet = MV_CC_RegisterBuffer(handle, stThreadInfo.pBuffer[i], nPayloadSize, NULL);
            if (MV_OK != nRet)
            {
                printf("%d MV_CC_RegisterBuffer size %d failed! %#x\n", i, nPayloadSize,nRet);
                break;
            }
        }

		if (MV_OK != nRet)
		{
			break;
		}

        nRet = cuMemAlloc(&stThreadInfo.cudaBuffer, nPayloadSize);
        if (CUDA_SUCCESS != nRet)
        {
            printf("cuMemAlloc buffer failed! %#x", nRet);
            break;
        }

        nRet = dvpCreateGPUCUDADevicePtr(stThreadInfo.cudaBuffer, &stThreadInfo.hGpuObjectHandle);
        if (DVP_STATUS_OK != nRet)
        {
            printf("dvpCreateGPUCUDADevicePtr failed! %#x", nRet);
            break;
        }


        SetupSyncObject(&stThreadInfo.stGpuSync);

        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        stThreadInfo.hCamHandle = handle;
        stThreadInfo.cuCtx = cuCtx;

        lock_create(&g_csLock);

		///< 创建两个线程，提高性能，减少丢帧
        unsigned int nThreadID = 0;
        void* hThreadHandle = (void*)_beginthreadex(NULL, 0, WorkThread, &stThreadInfo, 0, &nThreadID);
        if (NULL == hThreadHandle)
        {
            break;
        }

        unsigned int nCudaThreadID = 0;
        void* hCudaThreadHandle = (void*)_beginthreadex(NULL, 0, GpuThread, &stThreadInfo, 0, &nCudaThreadID);
        if (NULL == hCudaThreadHandle)
        {
            break;
        }

        printf("Press a key to stop grabbing.\n");
        WaitForKeyPress();

        g_bExit = true;
        Sleep(1000);

        WaitForSingleObject(hCudaThreadHandle, INFINITE);
        CloseHandle(hCudaThreadHandle);
        hCudaThreadHandle = NULL;


        WaitForSingleObject(hThreadHandle, INFINITE);
        CloseHandle(hThreadHandle);
        hThreadHandle = NULL;

        lock_destroy(&g_csLock);

        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        handle = NULL;
    } while (0);

    if (handle != NULL)
    {
        MV_CC_DestroyHandle(handle);
        handle = NULL;
    }


    // ch:反初始化SDK | en:Finalize SDK
    MV_CC_Finalize();


    // ch:撤销并释放注册的缓存 | en:Revoke and release buffer
    for (unsigned int i = 0; i < BUFFER_NUMBER; i++)
    {
        if (NULL != stThreadInfo.hSysMemHandle[i])
        {
            dvpUnbindFromCUDACtx(stThreadInfo.hSysMemHandle[i]);
            dvpDestroyBuffer(stThreadInfo.hSysMemHandle[i]);

            stThreadInfo.hSysMemHandle[i] = NULL;
        }

        if (NULL != stThreadInfo.stSysMemSync[i].hSyncObj)
        {
            dvpFreeSyncObject(stThreadInfo.stSysMemSync[i].hSyncObj);

            stThreadInfo.stSysMemSync[i].hSyncObj = NULL;
        }

		if (NULL != stThreadInfo.stSysMemSync[i].pnSemOrg)
		{
			free((void*)stThreadInfo.stSysMemSync[i].pnSemOrg);
			stThreadInfo.stSysMemSync[i].pnSemOrg = NULL;
		}


		if (stThreadInfo.pBuffer[i])
		{
			VirtualFree(stThreadInfo.pBuffer[i], 0, MEM_RELEASE);
			stThreadInfo.pBuffer[i] = NULL;
		}
    }

    if (NULL != stThreadInfo.hGpuObjectHandle)
    {
        dvpFreeBuffer(stThreadInfo.hGpuObjectHandle);
        stThreadInfo.hGpuObjectHandle = NULL;
    }

    if (NULL != stThreadInfo.stGpuSync.hSyncObj)
    {
        dvpFreeSyncObject(stThreadInfo.stGpuSync.hSyncObj);

        stThreadInfo.stGpuSync.hSyncObj = NULL;
    }

	if (NULL != stThreadInfo.stGpuSync.pnSemOrg)
	{
		free((void*)stThreadInfo.stGpuSync.pnSemOrg);
		stThreadInfo.stGpuSync.pnSemOrg = NULL;
	}


    if (0 != stThreadInfo.cudaBuffer)
    {
        cuMemFree(stThreadInfo.cudaBuffer);
        stThreadInfo.cudaBuffer = 0;
    }

    DeinitCuda(cuCtx);    

    printf("Press a key to exit.\n");
    WaitForKeyPress();

    return 0;
}

