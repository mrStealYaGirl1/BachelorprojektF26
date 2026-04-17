import AsyncStorage from '@react-native-async-storage/async-storage'
import type { ImuSample, MetaPacket } from './imuPackets'

export const IMU_EVENTS_STORAGE_KEY = 'vonpod:imu-events'

export type StoredImuEvent = {
  id: string
  localEventId?: number
  eventId: number
  savedAt: string
  sampleCount: number
  samples: ImuSample[]
  meta?: MetaPacket
}

export async function loadStoredEvents(): Promise<StoredImuEvent[]> {
  const raw = await AsyncStorage.getItem(IMU_EVENTS_STORAGE_KEY)
  if (!raw) {
    return []
  }

  return JSON.parse(raw) as StoredImuEvent[]
}

export function getHighestLocalEventId(events: StoredImuEvent[]): number {
  let highest = 0

  for (const event of events) {
    if (typeof event.localEventId === 'number' && Number.isFinite(event.localEventId)) {
      highest = Math.max(highest, event.localEventId)
    }
  }

  return highest
}

export async function saveStoredEvents(events: StoredImuEvent[]): Promise<void> {
  await AsyncStorage.setItem(IMU_EVENTS_STORAGE_KEY, JSON.stringify(events))
}

export async function clearStoredEvents(): Promise<void> {
  await AsyncStorage.removeItem(IMU_EVENTS_STORAGE_KEY)
}
