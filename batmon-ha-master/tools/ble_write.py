"""
Service Explorer
----------------

An example showing how to access and print out the services, characteristics and
descriptors of a connected GATT server.

Created on 2019-03-25 by hbldh <henrik.blidh@nedomkull.com>

"""

import sys
import platform
import asyncio
import logging

from bleak import BleakClient

logger = logging.getLogger(__name__)

ADDRESS = (
    "20:A1:11:02:23:45"
    if platform.system() != "Darwin"
    #else 'D4EBFBE4-8776-DEC0-CB23-BBA4963568AD' # ANT-BLE20PHUB
    else 'CA1E6A7D-DE32-8905-8EA2-F0574284F4E0' # clone esp32
)

async def main(address):
    logger.info('Connecting %s', address)
    async with BleakClient(address) as client:
        logger.info(f"Connected: {client.is_connected}")
        while client.is_connected:
            input('Press Enter to continue...')
            await client.start_notify('0000ffe1-0000-1000-8000-00805f9b34fb', print)
            #await client.write_gatt_char('0000ffe1-0000-1000-8000-00805f9b34fb', b'~\xa1\x01\x00\x00\xbe\x18U\xaaU')
            #print('wrote')




if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main(sys.argv[1] if len(sys.argv) == 2 else ADDRESS))