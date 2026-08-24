"""验证 macOS 上 libusb 能否接管 EVK4。

这是 OpenEB 能否在 Mac 上用起来的前提：ioreg 显示设备被枚举为通用的
IOUSBHostDevice、没有被专有驱动 claim，理论上 libusb 可以接管。但 macOS 对
USB 访问有自己的限制，必须实测。

在花两小时编译整个 SDK 之前先跑这个 —— 如果这里就失败，编译也是白费。
"""
import sys

import usb.core
import usb.util

VID, PID = 0x04B4, 0x00F5   # Cypress FX3，EVK4 用的控制器

dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    print("未找到 EVK4 (04b4:00f5)")
    sys.exit(1)

print("找到设备:")
try:
    print(f"  厂商:   {usb.util.get_string(dev, dev.iManufacturer)}")
    print(f"  产品:   {usb.util.get_string(dev, dev.iProduct)}")
    print(f"  序列号: {usb.util.get_string(dev, dev.iSerialNumber)}")
except Exception as exc:
    print(f"  (读取字符串描述符失败: {exc})")

print(f"  总线/地址: {dev.bus}/{dev.address}")
print(f"  USB 版本:  {dev.bcdUSB >> 8}.{(dev.bcdUSB >> 4) & 0xF}")
print(f"  速度码:    {getattr(dev, 'speed', '未知')}")

cfg = dev.get_active_configuration()
print(f"\n配置: {cfg.bConfigurationValue}, 接口数 {cfg.bNumInterfaces}")
for intf in cfg:
    print(f"  接口 {intf.bInterfaceNumber} alt {intf.bAlternateSetting}: "
          f"class=0x{intf.bInterfaceClass:02x} 端点数 {intf.bNumEndpoints}")
    for ep in intf:
        d = "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) else "OUT"
        t = ("控制", "同步", "批量", "中断")[usb.util.endpoint_type(ep.bmAttributes)]
        print(f"      端点 0x{ep.bEndpointAddress:02x} {d:3} {t}  "
              f"最大包 {ep.wMaxPacketSize}")

# 关键测试：能否 claim 接口。这是 libusb 真正接管设备的动作。
print()
intf0 = cfg[(0, 0)].bInterfaceNumber
try:
    if dev.is_kernel_driver_active(intf0):
        print(f"接口 {intf0} 被内核驱动占用 —— 需要先 detach")
    else:
        print(f"接口 {intf0} 未被内核驱动占用")
except NotImplementedError:
    print("macOS 不支持 is_kernel_driver_active（正常，非错误）")

try:
    usb.util.claim_interface(dev, intf0)
    print(f"成功 claim 接口 {intf0}")
    usb.util.release_interface(dev, intf0)
    print("已释放")
    print("\n结论：libusb 可以接管 EVK4，OpenEB 的 libusb 路径在 macOS 上可行。")
except Exception as exc:
    print(f"claim 接口失败: {exc}")
    print("\n结论：libusb 无法接管设备，需要进一步排查权限或驱动占用。")
    sys.exit(2)
