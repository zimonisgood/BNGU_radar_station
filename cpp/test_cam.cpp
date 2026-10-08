#include <iostream>
#include <cstring>
#include "MvCameraControl.h"

int main()
{
    void* handle = nullptr;
    MV_CC_DEVICE_INFO_LIST stDevList;
    memset(&stDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDevList);
    std::cout << "Enum ret:" << ret << ", device count:" << stDevList.nDeviceNum << std::endl;
    if(stDevList.nDeviceNum == 0)
    {
        std::cout << "No camera found!" << std::endl;
        return -1;
    }
    ret = MV_CC_CreateHandle(&handle, stDevList.pDeviceInfo[0]);
    std::cout << "CreateHandle ret:" << ret << std::endl;
    ret = MV_CC_OpenDevice(handle);
    std::cout << "OpenDevice ret:" << ret << std::endl;
    ret = MV_CC_StartGrabbing(handle);
    std::cout << "StartGrabbing ret:" << ret << std::endl;
    std::cout << "Grab running, press Enter to exit..." << std::endl;
    std::getchar();
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
