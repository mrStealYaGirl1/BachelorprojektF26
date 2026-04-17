import AsyncStorage from '@react-native-async-storage/async-storage'

export const TRAINING_SESSIONS_STORAGE_KEY = 'vonpod:training-sessions'

export type StoredTrainingSession = {
  id: string
  userId: string | null
  startedAt: string
  endedAt: string
  durationSeconds: number
  puttEventIds: string[]
  puttsCount: number
  createdAt: string
  activityName?: string
}

export async function loadStoredTrainingSessions(): Promise<StoredTrainingSession[]> {
  const raw = await AsyncStorage.getItem(TRAINING_SESSIONS_STORAGE_KEY)
  if (!raw) {
    return []
  }

  return JSON.parse(raw) as StoredTrainingSession[]
}

export async function saveStoredTrainingSessions(
  sessions: StoredTrainingSession[]
): Promise<void> {
  await AsyncStorage.setItem(TRAINING_SESSIONS_STORAGE_KEY, JSON.stringify(sessions))
}

export async function appendStoredTrainingSession(
  session: StoredTrainingSession
): Promise<void> {
  const existing = await loadStoredTrainingSessions()
  await saveStoredTrainingSessions([...existing, session])
}

export async function clearStoredTrainingSessions(): Promise<void> {
  await AsyncStorage.removeItem(TRAINING_SESSIONS_STORAGE_KEY)
}

export async function updateStoredTrainingSession(
  sessionId: string,
  updates: Partial<StoredTrainingSession>
): Promise<StoredTrainingSession[]> {
  const sessions = await loadStoredTrainingSessions()
  const nextSessions = sessions.map((session) =>
    session.id === sessionId ? { ...session, ...updates } : session
  )
  await saveStoredTrainingSessions(nextSessions)
  return nextSessions
}
