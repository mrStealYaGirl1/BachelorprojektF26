import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  ActivityIndicator,
  Animated,
  Easing,
  FlatList,
  ImageBackground,
  Modal,
  Pressable,
  SafeAreaView,
  StyleSheet,
  Text,
  View,
} from 'react-native'
import { BlurView } from 'expo-blur'
import { NativeModulesProxy } from 'expo-modules-core'
import type { Device, Subscription } from 'react-native-ble-plx'
import Svg, { Circle, Defs, RadialGradient, Stop } from 'react-native-svg'
import {
  bleManager,
  connectAndPrepare,
  disconnectIfConnected,
  findServiceForCharacteristic,
  monitorCharacteristic,
  waitForPoweredOn,
} from '../../lib/ble'
import {
  decodeBleNotification,
  type ImuSample,
  type MetaPacket,
} from '../../lib/imuPackets'
import { addMockPuttEvent } from '../../lib/mockPutt'
import {
  getHighestLocalEventId,
  loadStoredEvents,
  saveStoredEvents,
  type StoredImuEvent,
} from '../../lib/imuStorage'
import Ionicons from '@expo/vector-icons/Ionicons'
import { useAuth } from '../../providers/AuthProvider'
import { useFocusEffect } from '@react-navigation/native'
import { fetchOwnProfile, type ProfileRow } from '../../lib/profile'

const GOLF_IMU_CHARACTERISTIC_UUID = '99887766-5544-3322-1100-ffeeddccbbaa'
const DEFAULT_EVENT_SAMPLE_COUNT = 1000
const CAN_RENDER_BLUR = Boolean((NativeModulesProxy as Record<string, unknown>).ExpoBlurView)
const ENABLE_SCAN_PREVIEW_MOCK = true
const HOME_BACKGROUND_IMAGE = require('../../../assets/images/background_homepage.png')

type FoundDeviceItem = {
  id: string
  name: string | null
  localName: string | null
  rssi: number | null
  device?: Device
  isMock?: boolean
  batteryText?: string
}

const MOCK_FOUND_DEVICES: FoundDeviceItem[] = [
  {
    id: 'mock-vonpod-182661',
    name: 'vonpod182661',
    localName: 'vonpod182661',
    rssi: -48,
    isMock: true,
    batteryText: '98%',
  },
  {
    id: 'mock-vonpod-159823',
    name: 'vonpod159823',
    localName: 'vonpod159823',
    rssi: -63,
    isMock: true,
    batteryText: '37%',
  },
]

const HomeScreen = () => {
  const { session } = useAuth()
  const [, setStatus] = useState('Ikke forbundet')
  const [isBusy, setIsBusy] = useState(false)
  const [device, setDevice] = useState<Device | null>(null)
  const [lastSample, setLastSample] = useState<string | null>(null)
  const [lastDecodedSample, setLastDecodedSample] = useState<ImuSample | null>(null)
  const [currentEventId, setCurrentEventId] = useState<number | null>(null)
  const [currentEventSamples, setCurrentEventSamples] = useState(0)
  const [receivedSamples, setReceivedSamples] = useState(0)
  const [bufferedEventsCount, setBufferedEventsCount] = useState(0)
  const [lastMeta, setLastMeta] = useState<MetaPacket | null>(null)
  const [errorText, setErrorText] = useState<string | null>(null)
  const [mockInfoText, setMockInfoText] = useState<string | null>(null)
  const [isConnectModalVisible, setIsConnectModalVisible] = useState(false)
  const [isSearchingDevices, setIsSearchingDevices] = useState(false)
  const [foundDevices, setFoundDevices] = useState<FoundDeviceItem[]>([])
  const [selectedDeviceLabel, setSelectedDeviceLabel] = useState<string | null>(null)
  const [isMockConnected, setIsMockConnected] = useState(false)
  const [profile, setProfile] = useState<ProfileRow | null>(null)
  const subscriptionRef = useRef<Subscription | null>(null)
  const eventBuffersRef = useRef<Map<number, ImuSample[]>>(new Map())
  const eventMetaRef = useRef<Map<number, MetaPacket>>(new Map())
  const isPersistingRef = useRef(false)
  const scanTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null)
  const waveValuesRef = useRef([
    new Animated.Value(0),
    new Animated.Value(0),
    new Animated.Value(0),
  ])
  const waveLoopsRef = useRef<Animated.CompositeAnimation[]>([])

  const isConnected = useMemo(() => !!device || isMockConnected, [device, isMockConnected])

  useFocusEffect(
    useCallback(() => {
      let active = true

      const loadProfile = async () => {
        if (!session?.user) return

        try {
          const data = await fetchOwnProfile(session.user.id)
          if (!active) return
          setProfile(data)
        } catch {
          if (!active) return
          setProfile(null)
        }
      }

      void loadProfile()

      return () => {
        active = false
      }
    }, [session?.user])
  )

  const user = session?.user
  const userMetadata = user?.user_metadata ?? {}

  const usernameFromMetadata =
    typeof userMetadata.username === 'string' && userMetadata.username.trim().length > 0
      ? userMetadata.username.trim()
      : null

  const firstName =
    typeof userMetadata.first_name === 'string' && userMetadata.first_name.trim().length > 0
      ? userMetadata.first_name.trim()
      : null

  const lastName =
    typeof userMetadata.last_name === 'string' && userMetadata.last_name.trim().length > 0
      ? userMetadata.last_name.trim()
      : null

  const fullName = firstName && lastName ? `${firstName} ${lastName}` : null

  const firstNameFromProfile = profile?.first_name?.trim() || null
  const lastNameFromProfile = profile?.last_name?.trim() || null
  const fullNameFromProfile =
    firstNameFromProfile && lastNameFromProfile
      ? `${firstNameFromProfile} ${lastNameFromProfile}`
      : null

  const fallbackUsername =
    profile?.username ??
    usernameFromMetadata ??
    (user?.email ? user.email.split('@')[0] : null) ??
    'vonpod-user'

  const displayName = fullNameFromProfile ?? fullName ?? fallbackUsername
  const displayHandle = `@${fallbackUsername}`

  useEffect(() => {
    return () => {
      subscriptionRef.current?.remove()
      subscriptionRef.current = null
      void persistBufferedEvents(false)

      if (device?.id) {
        disconnectIfConnected(device.id).catch(() => {
          // Avoid unhandled promise on unmount.
        })
      }

      bleManager.stopDeviceScan()
      if (scanTimeoutRef.current) {
        clearTimeout(scanTimeoutRef.current)
      }

      waveLoopsRef.current.forEach((loop) => loop.stop())
    }
  }, [device])

  useEffect(() => {
    waveLoopsRef.current.forEach((loop) => loop.stop())
    waveLoopsRef.current = []

    if (!isSearchingDevices) {
      waveValuesRef.current.forEach((value) => value.setValue(0))
      return
    }

    const loops = waveValuesRef.current.map((value, index) => {
      value.setValue(0)

      const loop = Animated.loop(
        Animated.sequence([
          Animated.delay(index * 420),
          Animated.timing(value, {
            toValue: 1,
            duration: 1500,
            easing: Easing.out(Easing.ease),
            useNativeDriver: true,
          }),
          Animated.timing(value, {
            toValue: 0,
            duration: 0,
            useNativeDriver: true,
          }),
        ])
      )

      loop.start()
      return loop
    })

    waveLoopsRef.current = loops

    return () => {
      loops.forEach((loop) => loop.stop())
    }
  }, [isSearchingDevices])

  const stopScanning = () => {
    bleManager.stopDeviceScan()
    if (scanTimeoutRef.current) {
      clearTimeout(scanTimeoutRef.current)
      scanTimeoutRef.current = null
    }
    setIsSearchingDevices(false)
  }

  const isMatchingVonpodDevice = (candidate: Device): boolean => {
    const name = `${candidate.name ?? ''} ${candidate.localName ?? ''}`.toLowerCase()
    return name.includes('golf_imu') || name.includes('vonpod')
  }

  const applyMockDevicesIfNeeded = () => {
    if (!ENABLE_SCAN_PREVIEW_MOCK || !__DEV__) return

    setFoundDevices((previous) => {
      if (previous.length > 0) return previous
      return MOCK_FOUND_DEVICES
    })
  }

  const startDeviceSearch = async () => {
    setErrorText(null)
    setFoundDevices([])
    setIsSearchingDevices(true)

    try {
      await waitForPoweredOn()

      bleManager.startDeviceScan(null, null, (scanError, scannedDevice) => {
        if (scanError) {
          setErrorText(scanError.message)
          applyMockDevicesIfNeeded()
          stopScanning()
          return
        }

        if (!scannedDevice) {
          return
        }

        if (!isMatchingVonpodDevice(scannedDevice)) {
          return
        }

        setFoundDevices((previous) => {
          const exists = previous.some((entry) => entry.id === scannedDevice.id)
          if (exists) {
            return previous
          }

          return [
            ...previous,
            {
              id: scannedDevice.id,
              name: scannedDevice.name ?? null,
              localName: scannedDevice.localName ?? null,
              rssi: typeof scannedDevice.rssi === 'number' ? scannedDevice.rssi : null,
              device: scannedDevice,
            },
          ]
        })
      })

      scanTimeoutRef.current = setTimeout(() => {
        applyMockDevicesIfNeeded()
        stopScanning()
      }, 8000)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke starte BLE-søgning'
      setErrorText(message)
      applyMockDevicesIfNeeded()
      stopScanning()
    }
  }

  const getExpectedSamplesForEvent = (eventId: number): number => {
    const meta = eventMetaRef.current.get(eventId)
    if (!meta) {
      return DEFAULT_EVENT_SAMPLE_COUNT
    }

    return meta.totalSamples > 0 ? meta.totalSamples : DEFAULT_EVENT_SAMPLE_COUNT
  }

  const isEventComplete = (eventId: number, sampleCount: number): boolean => {
    return sampleCount >= getExpectedSamplesForEvent(eventId)
  }

  const persistBufferedEvents = async (onlyCompleted: boolean): Promise<number> => {
    if (eventBuffersRef.current.size === 0 || isPersistingRef.current) return 0

    isPersistingRef.current = true

    try {
      const existing = await loadStoredEvents()
      const now = Date.now()
      let nextLocalEventId = getHighestLocalEventId(existing) + 1

      const entriesToSave = Array.from(eventBuffersRef.current.entries()).filter(
        ([eventId, samples]) =>
          samples.length > 0 && (!onlyCompleted || isEventComplete(eventId, samples.length))
      )

      const eventsToSave: StoredImuEvent[] = entriesToSave.map(([eventId, samples], index) => {
          const localEventId = nextLocalEventId
          nextLocalEventId += 1

          return {
            id: `${now}-${eventId}-${index}`,
            localEventId,
            eventId,
            savedAt: new Date().toISOString(),
            sampleCount: samples.length,
            samples,
            meta: eventMetaRef.current.get(eventId),
          }
        })

      if (eventsToSave.length === 0) return 0

      const merged = [...existing, ...eventsToSave]
      await saveStoredEvents(merged)

      for (const [eventId] of entriesToSave) {
        eventBuffersRef.current.delete(eventId)
        eventMetaRef.current.delete(eventId)
      }

      setBufferedEventsCount(eventBuffersRef.current.size)
      return eventsToSave.length
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke gemme IMU-data'
      setErrorText(message)
      return 0
    } finally {
      isPersistingRef.current = false
    }
  }

  const connectToDevice = async (selectedDevice: FoundDeviceItem) => {
    const targetName = selectedDevice.name ?? selectedDevice.localName ?? 'Unknown device'

    if (selectedDevice.isMock || !selectedDevice.device) {
      setSelectedDeviceLabel(targetName)
      setIsMockConnected(true)
      setErrorText(null)
      setStatus(`Forbundet til ${targetName} (mock)`)
      setIsConnectModalVisible(false)
      stopScanning()
      return
    }

    const foundDevice = selectedDevice.device
    stopScanning()
    setIsBusy(true)
    setErrorText(null)
    setMockInfoText(null)
    setStatus('Forbinder...')
    setReceivedSamples(0)
    setCurrentEventId(null)
    setCurrentEventSamples(0)
    setLastDecodedSample(null)
    setLastMeta(null)
    setIsMockConnected(false)
    setBufferedEventsCount(0)
    eventBuffersRef.current.clear()
    eventMetaRef.current.clear()

    try {
      const connectedDevice = await connectAndPrepare(foundDevice)
      setDevice(connectedDevice)
      setSelectedDeviceLabel(targetName)

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
              void persistBufferedEvents(true)
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

            if (isEventComplete(packet.eventId, updated.length)) {
              void persistBufferedEvents(true)
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
      setIsConnectModalVisible(false)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Ukendt BLE-fejl'
      setErrorText(message)
      setSelectedDeviceLabel(null)
      setIsMockConnected(false)
      setStatus('Ikke forbundet')
    } finally {
      setIsBusy(false)
    }
  }

  const openConnectModal = () => {
    setIsConnectModalVisible(true)
    void startDeviceSearch()
  }

  const disconnect = async () => {
    if (!device && !isMockConnected) return

    setIsBusy(true)
    setErrorText(null)

    try {
      if (device) {
        subscriptionRef.current?.remove()
        subscriptionRef.current = null
        await disconnectIfConnected(device.id)
        await persistBufferedEvents(false)
      }
      setDevice(null)
      setIsMockConnected(false)
      setSelectedDeviceLabel(null)
      setStatus('Afbrudt')
      setLastSample(null)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke afbryde forbindelse'
      setErrorText(message)
    } finally {
      setIsBusy(false)
    }
  }

  const addMockEventNow = async () => {
    setIsBusy(true)
    setErrorText(null)

    try {
      const event = await addMockPuttEvent()
      setMockInfoText(`Mock putt gemt: putt #${event.localEventId ?? '-'}`)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke oprette mock putt'
      setErrorText(message)
    } finally {
      setIsBusy(false)
    }
  }

  return (
    <ImageBackground
      source={HOME_BACKGROUND_IMAGE}
      resizeMode='contain'
      imageStyle={styles.backgroundImageAsset}
      style={styles.backgroundImage}
    >
      <SafeAreaView style={styles.container}>
        <Text style={styles.title}>vonpod</Text>

        <View style={styles.profileRow}>
          <View style={styles.avatarPlaceholder}>
            <Ionicons name='person' size={42} color='#a4a8ac' />
          </View>

          <View style={styles.profileTextWrap}>
            <Text numberOfLines={1} style={styles.profileNameText}>{displayName}</Text>
            <Text numberOfLines={1} style={styles.profileHandleText}>{displayHandle}</Text>
          </View>
        </View>

        <View style={styles.decodedSection}>
          <Text style={styles.sectionTitle}>Seneste sample (dekodet)</Text>
          <Text style={styles.sampleText}>
            {lastDecodedSample
              ? `SEQ:${lastDecodedSample.seq} T:${lastDecodedSample.tsMs} AX:${lastDecodedSample.ax} AY:${lastDecodedSample.ay} AZ:${lastDecodedSample.az} GX:${lastDecodedSample.gx} GY:${lastDecodedSample.gy} GZ:${lastDecodedSample.gz} | ACC:${lastDecodedSample.accelG.toFixed(2)}g GYRO:${lastDecodedSample.gyroDps.toFixed(1)}dps`
              : 'Ingen dekodede samples endnu'}
          </Text>
          {__DEV__ && (
            <Pressable
              onPress={() => void addMockEventNow()}
              disabled={isBusy}
              style={styles.mockActionButton}
            >
              <Text style={styles.mockActionButtonText}>Tilføj mock putt</Text>
            </Pressable>
          )}
          {!!mockInfoText && <Text style={styles.mockInfoText}>{mockInfoText}</Text>}
        </View>

        <View style={styles.bottomConnectArea}>
          {!!selectedDeviceLabel && isConnected && (
            <View style={styles.connectedHintRow}>
              <Ionicons name='bluetooth-outline' size={14} color='#276eea' />
              <Text style={styles.connectedHintText}>
                Connected to <Text style={styles.connectedHintDevice}>{selectedDeviceLabel}</Text>
              </Text>
            </View>
          )}

          <Pressable
            onPress={isConnected ? () => void disconnect() : openConnectModal}
            disabled={isBusy}
            style={styles.bleButton}>
            {isBusy ? (
              <Text style={styles.bleButtonText}>Arbejder...</Text>
            ) : (
              <View style={styles.bleButtonContent}>
                <Text style={styles.bleButtonText}>
                  {isConnected ? (
                    'Disconnect device'
                  ) : (
                    <>
                      Connect your <Text style={styles.bleButtonBrandText}>vonpod</Text>
                    </>
                  )}
                </Text>
                <Ionicons name='bluetooth-outline' size={22} color='#FFFFFF' style={styles.bleButtonIcon} />
              </View>
            )}
          </Pressable>
        </View>

        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}

        <Modal
          visible={isConnectModalVisible}
          transparent
          animationType='fade'
          onRequestClose={() => {
            stopScanning()
            setIsConnectModalVisible(false)
          }}
        >
          <View style={styles.modalBackdrop}>
            {CAN_RENDER_BLUR && <BlurView intensity={25} tint='dark' style={styles.modalBlur} />}
            <View style={styles.modalCard}>
              <View style={styles.modalHeader}>
                <Pressable
                  onPress={() => {
                    stopScanning()
                    setIsConnectModalVisible(false)
                  }}
                  style={styles.modalBackButton}
                >
                  <Text style={styles.modalBackButtonText}>‹</Text>
                </Pressable>
                <Text style={styles.modalTitle}>
                  Connect your <Text style={styles.modalTitleBrand}>vonpod</Text>
                </Text>
              </View>

              <View style={styles.modalDivider} />

              <View style={styles.searchCenterArea}>
                <Svg width={180} height={180} viewBox='0 0 180 180' style={styles.searchRadialFade}>
                  <Defs>
                    <RadialGradient id='bluetoothCoreFade' cx='50%' cy='50%' r='50%'>
                      <Stop offset='0%' stopColor='#e4ebf5' stopOpacity='0.38' />
                      <Stop offset='36%' stopColor='#dbe3ef' stopOpacity='0.22' />
                      <Stop offset='62%' stopColor='#cfd8e5' stopOpacity='0.12' />
                      <Stop offset='80%' stopColor='#c5cfdd' stopOpacity='0.05' />
                      <Stop offset='100%' stopColor='#bdc8d7' stopOpacity='0' />
                    </RadialGradient>
                  </Defs>
                  <Circle cx='90' cy='90' r='90' fill='url(#bluetoothCoreFade)' />
                </Svg>

                {waveValuesRef.current.map((value, index) => (
                  <Animated.View
                    key={`wave-${index}`}
                    pointerEvents='none'
                    style={[
                      styles.searchWaveRing,
                      {
                        opacity: value.interpolate({
                          inputRange: [0, 0.2, 1],
                          outputRange: [0, 0.5, 0],
                        }),
                        transform: [
                          {
                            scale: value.interpolate({
                              inputRange: [0, 1],
                              outputRange: [0.72, 1.8],
                            }),
                          },
                        ],
                      },
                    ]}
                  />
                ))}
    
                  <View style={styles.searchGlowCircleInner}>
                    <Ionicons name='bluetooth-outline' size={42} color='#e3ebff' />
                  </View>
              </View>

              <Text style={styles.searchStateText}>
                {isSearchingDevices ? 'Searching for devices...' : 'Devices found'}
              </Text>

              <View style={styles.deviceListSlot}>
                <FlatList
                  data={foundDevices}
                  keyExtractor={(item) => item.id}
                  contentContainerStyle={styles.deviceListContent}
                  ListEmptyComponent={
                    <Text style={[styles.deviceEmptyText, isSearchingDevices && styles.deviceEmptyHidden]}>
                      No devices found
                    </Text>
                  }
                  renderItem={({ item }) => (
                    <Pressable style={styles.deviceRow} onPress={() => void connectToDevice(item)}>
                      <Text style={styles.deviceNameText}>{item.name ?? item.localName ?? 'Unknown device'}</Text>
                      <View style={styles.deviceMetaRight}>
                        <Text style={styles.deviceMetaText}>
                          {item.batteryText ?? (typeof item.rssi === 'number' ? `${item.rssi} dBm` : '-')}
                        </Text>
                        <Text style={styles.deviceChevron}>›</Text>
                      </View>
                    </Pressable>
                  )}
                />
              </View>

              <View style={styles.modalActionSlot}>
                {isSearchingDevices ? (
                  <ActivityIndicator color='#8fb1ff' style={styles.searchSpinner} />
                ) : (
                  <Pressable style={styles.searchAgainButton} onPress={() => void startDeviceSearch()}>
                    <Text style={styles.searchAgainButtonText}>Search again</Text>
                  </Pressable>
                )}
              </View>
            </View>
          </View>
        </Modal>
      </SafeAreaView>
    </ImageBackground>
  )
}

const styles = StyleSheet.create({
  backgroundImage: {
    flex: 1,
  },
  backgroundImageAsset: {
    alignSelf: 'flex-start',
    marginTop: -410,
    width: '110%',
    marginLeft: -20,
  },
  container: {
    flex: 1,
    paddingHorizontal: 16,
    paddingTop: 24,
    margin: 16,
  },
  title: {
    fontSize: 42,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: { width: -1, height: 1 },
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#fff',
    marginBottom: 8,
  },
  profileRow: {
    marginTop: 26,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 16,
    paddingHorizontal: 18,
  },
  avatarPlaceholder: {
    width: 64,
    height: 64,
    borderRadius: 999,
    backgroundColor: '#dddfe1',
    alignItems: 'center',
    justifyContent: 'center',
  },
  profileTextWrap: {
    flex: 1,
    minWidth: 0,
  },
  profileNameText: {
    color: '#ffffff',
    fontSize: 22,
    fontFamily: 'RethinkSans_600SemiBold',
  },
  profileHandleText: {
    color: '#f0f0f0',
    fontSize: 16,
    marginTop: 2,
    fontFamily: 'RethinkSans_400Regular',
  },
  decodedSection: {
    marginTop: 8,
  },
  bottomConnectArea: {
    marginTop: 'auto',
    paddingBottom: 90,
    paddingHorizontal: 16,
  },
  bleButton: {
    backgroundColor: '#303030',
    marginTop: 8,
    minHeight: 45,
    width: '100%',
    borderRadius: 999,
    paddingHorizontal: 24,
    justifyContent: 'center',
  },
  bleButtonText: {
    color: '#FFFFFF',
    fontFamily: 'RethinkSans_400Regular',
    fontSize: 18,
    marginLeft: 46,
    paddingBottom: 3,
  },
  bleButtonContent: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  bleButtonIcon: {
    marginLeft: 36,
  },
  bleButtonBrandText: {
    fontFamily: 'Mitr_600SemiBold',
    fontSize: 21,
    color: '#FFFFFF',
  },
  connectedHintRow: {
    flexDirection: 'row',
    justifyContent: 'center',
    alignItems: 'center',
    gap: 4,
    marginBottom: 6,
    paddingHorizontal: 8,
    zIndex: 2,
  },
  connectedHintText: {
    fontSize: 14,
    color: '#276eea',
    textAlign: 'center',
    fontFamily: 'RethinkSans_400Regular',
  },
  connectedHintDevice: {
    color: '#276eea',
    fontFamily: 'Mitr_600SemiBold',
    fontSize: 15,
  },
  sectionTitle: {
    marginTop: 20,
    fontSize: 16,
    fontWeight: '600',
  },
  sampleText: {
    marginTop: 6,
    fontSize: 13,
    color: '#f5f5f5',
  },
  mockActionButton: {
    marginTop: 12,
    borderRadius: 999,
    backgroundColor: 'rgba(32, 32, 32, 0.72)',
    paddingHorizontal: 12,
    minHeight: 32,
    width: 160,
    justifyContent: 'center',
    alignSelf: 'center',
  },
  mockActionButtonText: {
    color: '#f7f7f7',
    fontSize: 13,
    fontFamily: 'RethinkSans_500Medium',
    alignSelf: 'center',
  },
  mockInfoText: {
    marginTop: 6,
    fontSize: 12,
    color: '#d7f0d7',
  },
  errorText: {
    marginTop: 12,
    color: '#b00020',
  },
  modalBackdrop: {
    flex: 1,
    backgroundColor: 'rgba(10, 10, 12, 0.45)',
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingTop: 120,
  },
  modalBlur: {
    ...StyleSheet.absoluteFillObject,
  },
  modalCard: {
    width: '100%',
    maxWidth: 360,
    borderRadius: 30,
    backgroundColor: '#303030',
    paddingHorizontal: 30,
    paddingBottom: 24,
    paddingTop: 14,
  },
  modalHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 10,
  },
  modalBackButton: {
    width: 30,
    height: 36,
  },
  modalBackButtonText: {
    color: '#f3f3f5',
    fontSize: 36,
    lineHeight: 36,
    fontWeight: '300',
  },
  modalTitle: {
    color: '#ffffff',
    fontSize: 20,
    fontFamily: 'RethinkSans_400Regular',
    justifyContent: 'center',
    alignItems: 'center',
    flex: 1,
    textAlign: 'center',
    paddingRight: 30,
    paddingBottom: 3,
  },
  modalTitleBrand: {
    fontFamily: 'Mitr_600SemiBold',
    color: '#ffffff',
    fontSize: 21,
  },
  modalDivider: {
    height: 1,
    backgroundColor: '#d0d0d0',
    opacity: 0.6,
  },
  searchCenterArea: {
    alignItems: 'center',
    justifyContent: 'center',
    width: 220,
    height: 220,
    alignSelf: 'center',
    position: 'relative',
    marginTop: -24,
  },
  searchRadialFade: {
    position: 'absolute',
    left: 20,
    top: 20,
  },
  searchWaveRing: {
    position: 'absolute',
    width: 84,
    height: 84,
    borderRadius: 55,
    borderWidth: 1.5,
    borderColor: 'rgba(171, 200, 247, 0.62)',
  },
  searchGlowCircleInner: {
    width: 84,
    height: 84,
    borderRadius: 42,
    backgroundColor: 'rgba(186, 199, 217, 0.05)',
    borderWidth: 2,
    borderColor: 'rgba(199, 219, 255, 0.82)',
    alignItems: 'center',
    justifyContent: 'center',
  },
  searchStateText: {
    color: '#f0f1f3',
    textAlign: 'center',
    fontSize: 20,
    fontFamily: 'RethinkSans_400Regular',
    marginTop: -30,
    marginBottom: 16,
  },
  deviceListContent: {
    paddingVertical: 2,
  },
  deviceListSlot: {
    height: 120,
  },
  deviceRow: {
    minHeight: 38,
    marginBottom: 8,
    borderRadius: 24,
    backgroundColor: '#4a4d52',
    paddingHorizontal: 14,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    overflow: 'visible',
  },
  deviceNameText: {
    color: '#f1f3f5',
    fontSize: 16,
    fontFamily: 'RethinkSans_400Regular',
    flex: 1,
    marginLeft: 6,
  },
  deviceMetaRight: {
    flexDirection: 'row',
    alignItems: 'center',
    marginLeft: 'auto',
    gap: 10,
  },
  deviceMetaText: {
    color: '#d8dbe0',
    fontSize: 14,
  },
  deviceChevron: {
    color: '#ebedf0',
    fontSize: 30,
    lineHeight: 32,
  },
  deviceEmptyText: {
    color: '#bfc2c8',
    textAlign: 'center',
    marginTop: 10,
    fontSize: 14,
  },
  deviceEmptyHidden: {
    opacity: 0,
  },
  searchSpinner: {
    alignSelf: 'center',
  },
  modalActionSlot: {
    minHeight: 58,
    justifyContent: 'center',
  },
  searchAgainButton: {
    marginTop: 0,
    minHeight: 50,
    borderRadius: 999,
    backgroundColor: '#5a75ff',
    alignItems: 'center',
    justifyContent: 'center',
  },
  searchAgainButtonText: {
    color: '#ffffff',
    fontSize: 20,
    fontFamily: 'RethinkSans_600SemiBold',
    marginHorizontal: 4,
  },
})

export default HomeScreen
