/* import { BleManager, Device, Subscription } from 'react-native-ble-plx'

export const bleManager = new BleManager()

export async function scanOnce(): Promise<Device> {
  return new Promise((resolve, reject) => {
    let done = false

    bleManager.startDeviceScan(null, null, (error, device) => {
      if (error) {
        bleManager.stopDeviceScan()
        reject(error)
        return
      }

      if (!device) return

      const match =
        device.name?.includes('GOLF_IMU') ||
        device.localName?.includes('GOLF_IMU')

      if (match) {
        done = true
        bleManager.stopDeviceScan()
        resolve(device)
      }
    })

    setTimeout(() => {
      if (!done) {
        bleManager.stopDeviceScan()
        reject(new Error('Ingen BLE-enhed fundet'))
      }
    }, 8000)
  })
}

export async function waitForPoweredOn(timeoutMs = 10000): Promise<void> {
  const current = await bleManager.state()
  if (current === 'PoweredOn') return

  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      subscription.remove()
      reject(new Error('Bluetooth blev ikke aktiv i tide'))
    }, timeoutMs)

    const subscription = bleManager.onStateChange((state) => {
      if (state === 'PoweredOn') {
        clearTimeout(timeout)
        subscription.remove()
        resolve()
      }
    }, true)
  })
}

export async function connectAndPrepare(device: Device): Promise<Device> {
  const connected = await device.connect({ timeout: 10000 })
  return connected.discoverAllServicesAndCharacteristics()
}

export function monitorCharacteristic(
  deviceId: string,
  serviceUuid: string,
  characteristicUuid: string,
  onValue: (base64Value: string) => void,
  onError?: (error: Error) => void
): Subscription {
  return bleManager.monitorCharacteristicForDevice(
    deviceId,
    serviceUuid,
    characteristicUuid,
    (error, characteristic) => {
      if (error) {
        onError?.(error)
        return
      }

      if (characteristic?.value) {
        onValue(characteristic.value)
      }
    }
  )
}

export async function findServiceForCharacteristic(
  deviceId: string,
  characteristicUuid: string
): Promise<string> {
  const wanted = characteristicUuid.toLowerCase()
  const services = await bleManager.servicesForDevice(deviceId)

  for (const service of services) {
    const characteristics = await bleManager.characteristicsForDevice(deviceId, service.uuid)
    const hasCharacteristic = characteristics.some(
      (characteristic) => characteristic.uuid.toLowerCase() === wanted
    )

    if (hasCharacteristic) {
      return service.uuid
    }
  }

  throw new Error(`Kunne ikke finde service for characteristic ${characteristicUuid}`)
}

export async function disconnectIfConnected(deviceId: string): Promise<void> {
  const isConnected = await bleManager.isDeviceConnected(deviceId)
  if (isConnected) {
    await bleManager.cancelDeviceConnection(deviceId)
  }
} 
*/

import { BleManager, Device, Subscription } from 'react-native-ble-plx'

export const bleManager = new BleManager()

export const SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
export const TX_CHARACTERISTIC_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

export async function scanOnce(): Promise<Device> {
  return new Promise((resolve, reject) => {
    let done = false

    console.log("🔍 Starting BLE scan...")

    bleManager.startDeviceScan(
      [SERVICE_UUID], // FILTRER DIREKTE PÅ DIN DEVICE
      null,
      (error, device) => {
        if (error) {
          bleManager.stopDeviceScan()
          reject(error)
          return
        }

        if (!device) return

        // DEBUG – så I kan se ALT hvad der bliver fundet
        console.log("FOUND DEVICE:", {
          name: device.name,
          localName: device.localName,
          id: device.id,
          serviceUUIDs: device.serviceUUIDs
        })

        // MATCH: enten UUID eller navn (fallback)
        const match =
          device.serviceUUIDs?.includes(SERVICE_UUID) ||
          device.name?.includes('GOLF_IMU') ||
          device.localName?.includes('GOLF_IMU')

        if (match) {
          console.log("✅ MATCH FOUND:", device.id)

          done = true
          bleManager.stopDeviceScan()
          resolve(device)
        }
      }
    )

    setTimeout(() => {
      if (!done) {
        bleManager.stopDeviceScan()
        reject(new Error('Ingen BLE-enhed fundet'))
      }
    }, 8000)
  })
}

export async function waitForPoweredOn(timeoutMs = 10000): Promise<void> {
  const current = await bleManager.state()
  if (current === 'PoweredOn') return

  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      subscription.remove()
      reject(new Error('Bluetooth blev ikke aktiv i tide'))
    }, timeoutMs)

    const subscription = bleManager.onStateChange((state) => {
      if (state === 'PoweredOn') {
        clearTimeout(timeout)
        subscription.remove()
        resolve()
      }
    }, true)
  })
}

export async function connectAndPrepare(device: Device): Promise<Device> {
  console.log("BLE connect attempt:", {
    id: device.id,
    name: device.name,
    localName: device.localName,
    serviceUUIDs: device.serviceUUIDs,
  })

  try {
    const alreadyConnected = await bleManager.isDeviceConnected(device.id)

    if (alreadyConnected) {
      console.log("Device already connected, cancelling old connection first")
      await bleManager.cancelDeviceConnection(device.id)
    }

    console.log("Connecting to:", device.id)

    const connected = await bleManager.connectToDevice(device.id, {
      timeout: 10000,
      autoConnect: false,
    })

    console.log("Connected:", connected.id)

    try {
      await connected.requestMTU(247)
      console.log("MTU request done/skipped")
    } catch (mtuError) {
      console.log("MTU request failed:", mtuError)
    }

    console.log("Discovering services...")
    const discovered = await connected.discoverAllServicesAndCharacteristics()

    console.log("Discovery done")

    return discovered
  } catch (error) {
    console.log("BLE connectAndPrepare failed:", JSON.stringify(error, null, 2))
    throw error
  }
}

export function monitorCharacteristic(
  deviceId: string,
  serviceUuid: string,
  characteristicUuid: string,
  onValue: (base64Value: string) => void,
  onError?: (error: Error) => void
): Subscription {
  console.log("Subscribing to notifications...")

  return bleManager.monitorCharacteristicForDevice(
    deviceId,
    serviceUuid,
    characteristicUuid,
    (error, characteristic) => {
      if (error) {
        console.error("Notify error:", error)
        onError?.(error)
        return
      }

      if (characteristic?.value) {
        console.log("Data received")
        onValue(characteristic.value)
      }
    }
  )
}

export async function findServiceForCharacteristic(
  deviceId: string,
  characteristicUuid: string
): Promise<string> {
  const wanted = characteristicUuid.toLowerCase()
  const services = await bleManager.servicesForDevice(deviceId)

  for (const service of services) {
    const characteristics = await bleManager.characteristicsForDevice(deviceId, service.uuid)

    const hasCharacteristic = characteristics.some(
      (characteristic) => characteristic.uuid.toLowerCase() === wanted
    )

    if (hasCharacteristic) {
      return service.uuid
    }
  }

  throw new Error("Kunne ikke finde service for characteristic ${characteristicUuid}")
}

export async function disconnectIfConnected(deviceId: string): Promise<void> {
  const isConnected = await bleManager.isDeviceConnected(deviceId)

  if (isConnected) {
    console.log("Disconnecting...")
    await bleManager.cancelDeviceConnection(deviceId)
  }
}

export async function logServicesAndCharacteristics(deviceId: string): Promise<void> {
  const services = await bleManager.servicesForDevice(deviceId)

  for (const service of services) {
    console.log("SERVICE:", service.uuid)

    const characteristics = await bleManager.characteristicsForDevice(
      deviceId,
      service.uuid
    )

    for (const characteristic of characteristics) {
      console.log("  CHAR:", {
        uuid: characteristic.uuid,
        isReadable: characteristic.isReadable,
        isWritableWithResponse: characteristic.isWritableWithResponse,
        isWritableWithoutResponse: characteristic.isWritableWithoutResponse,
        isNotifiable: characteristic.isNotifiable,
        isIndicatable: characteristic.isIndicatable,
      })
    }
  }
}


