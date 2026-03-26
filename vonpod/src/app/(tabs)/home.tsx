import React, { useEffect, useMemo, useRef, useState } from 'react'
import { Button, Pressable, SafeAreaView, StyleSheet, Text, View } from 'react-native'
import type { Device, Subscription } from 'react-native-ble-plx'
import {
  connectAndPrepare,
  disconnectIfConnected,
  findServiceForCharacteristic,
  monitorCharacteristic,
  scanOnce,
  waitForPoweredOn,
} from '../../lib/ble'
import {
  decodeBleNotification,
  type ImuSample,
  type MetaPacket,
} from '../../lib/imuPackets'
import {
  loadStoredEvents,
  saveStoredEvents,
  type StoredImuEvent,
} from '../../lib/imuStorage'

const GOLF_IMU_CHARACTERISTIC_UUID = '99887766-5544-3322-1100-ffeeddccbbaa'

const HomeScreen = () => {
  const [status, setStatus] = useState('Ikke forbundet')
  const [isBusy, setIsBusy] = useState(false)
  const [device, setDevice] = useState<Device | null>(null)
  const [lastSample, setLastSample] = useState<string | null>(null)
  const [lastDecodedSample, setLastDecodedSample] = useState<ImuSample | null>(null)
  const [currentEventId, setCurrentEventId] = useState<number | null>(null)
  const [currentEventSamples, setCurrentEventSamples] = useState(0)
  const [receivedSamples, setReceivedSamples] = useState(0)
  const [savedEventsCount, setSavedEventsCount] = useState(0)
  const [bufferedEventsCount, setBufferedEventsCount] = useState(0)
  const [saveInfoText, setSaveInfoText] = useState<string | null>(null)
  const [lastMeta, setLastMeta] = useState<MetaPacket | null>(null)
  const [errorText, setErrorText] = useState<string | null>(null)
  const subscriptionRef = useRef<Subscription | null>(null)
  const eventBuffersRef = useRef<Map<number, ImuSample[]>>(new Map())
  const eventMetaRef = useRef<Map<number, MetaPacket>>(new Map())

  const canDisconnect = useMemo(() => !!device, [device])

  useEffect(() => {
    const loadSavedCount = async () => {
      try {
        const parsed = await loadStoredEvents()
        setSavedEventsCount(parsed.length)
      } catch {
        setSavedEventsCount(0)
      }
    }

    void loadSavedCount()
  }, [])

  useEffect(() => {
    return () => {
      subscriptionRef.current?.remove()
      subscriptionRef.current = null
      void persistBufferedEvents()

      if (device?.id) {
        disconnectIfConnected(device.id).catch(() => {
          // Avoid unhandled promise on unmount.
        })
      }
    }
  }, [device])

  const persistBufferedEvents = async (): Promise<number> => {
    if (eventBuffersRef.current.size === 0) return 0

    try {
      const existing = await loadStoredEvents()
      const now = Date.now()

      const eventsToSave: StoredImuEvent[] = Array.from(eventBuffersRef.current.entries())
        .filter(([, samples]) => samples.length > 0)
        .map(([eventId, samples], index) => ({
          id: `${now}-${eventId}-${index}`,
          eventId,
          savedAt: new Date().toISOString(),
          sampleCount: samples.length,
          samples,
          meta: eventMetaRef.current.get(eventId),
        }))

      if (eventsToSave.length === 0) return 0

      const merged = [...existing, ...eventsToSave]
      await saveStoredEvents(merged)

      eventBuffersRef.current.clear()
      eventMetaRef.current.clear()
      setBufferedEventsCount(0)
      setSavedEventsCount(merged.length)
      return eventsToSave.length
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke gemme IMU-data'
      setErrorText(message)
      return 0
    }
  }

  const saveEventsNow = async () => {
    setIsBusy(true)
    setErrorText(null)

    try {
      const savedNow = await persistBufferedEvents()
      if (savedNow > 0) {
        setSaveInfoText(`Gemte ${savedNow} event(s) lokalt`)
      } else {
        setSaveInfoText('Ingen nye events at gemme endnu')
      }
    } finally {
      setIsBusy(false)
    }
  }

  const startBleFlow = async () => {
    setIsBusy(true)
    setErrorText(null)
    setStatus('Tjekker Bluetooth...')
    setReceivedSamples(0)
    setCurrentEventId(null)
    setCurrentEventSamples(0)
    setLastDecodedSample(null)
    setLastMeta(null)
    setBufferedEventsCount(0)
    setSaveInfoText(null)
    eventBuffersRef.current.clear()
    eventMetaRef.current.clear()

    try {
      await waitForPoweredOn()

      setStatus('Scanner efter GOLF_IMU...')
      const foundDevice = await scanOnce()

      setStatus('Forbinder...')
      const connectedDevice = await connectAndPrepare(foundDevice)
      setDevice(connectedDevice)

      const serviceUuid = await findServiceForCharacteristic(
        connectedDevice.id,
        GOLF_IMU_CHARACTERISTIC_UUID
      )

      subscriptionRef.current?.remove()
      subscriptionRef.current = monitorCharacteristic(
        connectedDevice.id,
        serviceUuid,
        GOLF_IMU_CHARACTERISTIC_UUID,
        (value) => {
          setLastSample(value)

          try {
            const decoded = decodeBleNotification(value)

            if (decoded.kind === 'meta') {
              eventMetaRef.current.set(decoded.packet.eventId, decoded.packet)
              setLastMeta(decoded.packet)
              setCurrentEventId(decoded.packet.eventId)
              setErrorText(null)
              return
            }

            const packet = decoded.packet

            const previous = eventBuffersRef.current.get(packet.eventId) ?? []
            const updated = [...previous, ...packet.samples]
            eventBuffersRef.current.set(packet.eventId, updated)
            setBufferedEventsCount(eventBuffersRef.current.size)

            setCurrentEventId(packet.eventId)
            setCurrentEventSamples(updated.length)
            setReceivedSamples((prev) => prev + packet.samples.length)

            if (packet.samples.length > 0) {
              setLastDecodedSample(packet.samples[packet.samples.length - 1])
            }

            setErrorText(null)
          } catch (error) {
            const message =
              error instanceof Error
                ? `Decode-fejl: ${error.message}`
                : 'Decode-fejl ved BLE-pakke'
            setErrorText(message)
          }
        },
        (error) => {
          setErrorText(error.message)
        }
      )

      setStatus(`Forbundet til ${connectedDevice.name ?? connectedDevice.localName ?? 'GOLF_IMU'}`)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Ukendt BLE-fejl'
      setErrorText(message)
      setStatus('Ikke forbundet')
    } finally {
      setIsBusy(false)
    }
  }

  const disconnect = async () => {
    if (!device) return

    setIsBusy(true)
    setErrorText(null)

    try {
      subscriptionRef.current?.remove()
      subscriptionRef.current = null
      await disconnectIfConnected(device.id)
      await persistBufferedEvents()
      setDevice(null)
      setStatus('Afbrudt')
      setLastSample(null)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke afbryde forbindelse'
      setErrorText(message)
    } finally {
      setIsBusy(false)
    }
  }

  return (
    <SafeAreaView style={styles.container}>
      <Text style={styles.title}>Home</Text>
      <Text style={styles.statusLabel}>Status</Text>
      <Text style={styles.statusText}>{status}</Text>

      <View style={styles.buttonRow}>
        <Pressable
          onPress={startBleFlow}
          disabled={isBusy}
          style={styles.bleButton}>
          <Text style={styles.bleButtonText}> 
            {isBusy ? 'Arbejder...' : 'Connect your vonpod'}
          </Text>
        </Pressable>
      </View>

      <View style={styles.buttonRow}>
        <Button title='Afbryd' onPress={disconnect} disabled={isBusy || !canDisconnect} />
      </View>

      <View style={styles.buttonRow}>
        <Button
          title='Gem events'
          onPress={saveEventsNow}
          disabled={isBusy || bufferedEventsCount === 0}
        />
      </View>

      <Text style={styles.sectionTitle}>Seneste sample (base64)</Text>
      <Text style={styles.sampleText}>{lastSample ?? 'Ingen data endnu'}</Text>

      <Text style={styles.sectionTitle}>Dekodet status</Text>
      <Text style={styles.sampleText}>Nuv. event: {currentEventId ?? '-'}</Text>
      <Text style={styles.sampleText}>Samples i event: {currentEventSamples}</Text>
      <Text style={styles.sampleText}>Samples modtaget i alt: {receivedSamples}</Text>
      <Text style={styles.sampleText}>Events i buffer: {bufferedEventsCount}</Text>
      <Text style={styles.sampleText}>Gemte events: {savedEventsCount}</Text>
      {!!saveInfoText && <Text style={styles.infoText}>{saveInfoText}</Text>}

      <Text style={styles.sectionTitle}>Seneste sample (dekodet)</Text>
      <Text style={styles.sampleText}>
        {lastDecodedSample
          ? `SEQ:${lastDecodedSample.seq} T:${lastDecodedSample.tsMs} AX:${lastDecodedSample.ax} AY:${lastDecodedSample.ay} AZ:${lastDecodedSample.az} GX:${lastDecodedSample.gx} GY:${lastDecodedSample.gy} GZ:${lastDecodedSample.gz} | ACC:${lastDecodedSample.accelG.toFixed(2)}g GYRO:${lastDecodedSample.gyroDps.toFixed(1)}dps`
          : 'Ingen dekodede samples endnu'}
      </Text>

      <Text style={styles.sectionTitle}>Seneste META</Text>
      <Text style={styles.sampleText}>
        {lastMeta
          ? `EV:${lastMeta.eventId} swing:${lastMeta.swingId} fs:${lastMeta.sampleRateHz}Hz total:${lastMeta.totalSamples} impact_idx:${lastMeta.impactIndexInEvent}`
          : 'Ingen META modtaget endnu'}
      </Text>

      {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingHorizontal: 16,
    paddingTop: 24,
    margin: 16,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
    marginBottom: 16,
  },
  statusLabel: {
    fontSize: 16,
    fontWeight: '600',
  },
  statusText: {
    fontSize: 16,
    marginTop: 4,
    marginBottom: 16,
  },
  buttonRow: {
    marginBottom: 12,
  },
  bleButton: {
    backgroundColor: '#303030',
    marginTop: 8,
    minHeight: 52,
    minWidth: 180,
    borderRadius: 999,
    alignItems: 'center',
    justifyContent: 'center',
  },
  bleButtonText: {
    color: '#FFFFFF',
    fontSize: 18,
    fontWeight: '600',
  },
  sectionTitle: {
    marginTop: 20,
    fontSize: 16,
    fontWeight: '600',
  },
  sampleText: {
    marginTop: 6,
    fontSize: 13,
  },
  errorText: {
    marginTop: 16,
    color: '#b00020',
  },
  infoText: {
    marginTop: 8,
    color: '#1b5e20',
  },
})

export default HomeScreen
