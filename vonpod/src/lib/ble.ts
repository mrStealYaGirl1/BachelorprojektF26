import { BleManager, Device, Subscription } from 'react-native-ble-plx'

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