import asyncio
import logging

from bleak import BleakScanner

logger = logging.getLogger(__name__)

async def bt_discovery(timeout: int = 5, adapter=None, **kwargs):
    ad = adapter or 'default'
    logger.info('BT Discovery (%d seconds, adapter=%s):', timeout, adapter or 'default')
    scanner = BleakScanner(adapter=adapter, **kwargs)
    await scanner.start()
    await asyncio.sleep(timeout)
    await scanner.stop()
    if hasattr(scanner, 'discovered_devices_and_advertisement_data'):
        devices = scanner.discovered_devices_and_advertisement_data
        addr_len = (max(len(d.address) for d, a in devices.values()) + 1) if devices else 20
        if not devices:
            logger.info(' - no devices found - ')
        else:
            logger.info("%s %*s %26s %4s", ad, addr_len, 'addr', 'name', 'rssi')
        for d, a in sorted(devices.values(), key=lambda t: t[0].address):
            logger.info("%s %*s %26s %4s", ad, addr_len, d.address, d.name, a.rssi)
        return [d for d, a in devices.values()]
    else:
        devices = scanner.discovered_devices
        if not devices:
            logger.info(' - no devices found - ')
        else:
            logger.info("BT %18s %26s", 'addr', 'name')
        for d in devices:
            logger.info("BT %s %26s", d.address, d.name)
        return devices
    
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(bt_discovery())   
