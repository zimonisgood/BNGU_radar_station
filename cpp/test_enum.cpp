#include <iostream>
#include "/opt/MVS/include/MvCameraControl.h"

int main()
{
    void* handle = nullptr;
    MV_CC_DEVICE_INFO_LIST devList = {0};

    // ==========关键：指定网卡enp129s0，只在这张网卡上搜索GigE相机==========
    int ret = MV_CC_SetEnumInterface("enp129s0");
    std::cout << "MV_CC_SetEnumInterface ret=" << ret << std::endl;

    // 只枚举GigE设备
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE, &devList);
    std::cout << "EnumDevices ret=" << ret << std::endl;
    std::cout << "找到相机数量：" << devList.nDeviceNum << std::endl;

    if(devList.nDeviceNum <= 0)
    {
        std::cerr << "未发现GigE相机" << std::endl;
        return -1;
    }

    // 使用第0台相机创建设备句柄
    ret = MV_CC_CreateHandle(&handle, devList.pDeviceInfo[0]);
    std::cout << "CreateHandle ret=" << ret << std::endl;

    ret = MV_CC_OpenDevice(handle);
    std::cout << "OpenDevice ret=" << ret << std::endl;
    if(ret == 0)
    {
        std::cout << "✅ 相机打开成功！" << std::endl;
    }

    // 设置GigE包大小1400
    MV_CC_SetIntValue(handle, "GevSCPSPacketSize", 1400);
    // 关闭触发
    MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);

    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    return 0;
}
