import React, { createContext, useContext, useEffect, useMemo, useRef, useState } from 'react'
import { useAuth } from './AuthProvider'
import { loadStoredEvents } from '../lib/imuStorage'
import {
  appendStoredTrainingSession,
  type StoredTrainingSession,
} from '../lib/trainingStorage'
import { syncTrainingSessionToSupabase } from '../lib/trainingSync'

type TrainingContextType = {
  isTraining: boolean
  startedAt: string | null
  elapsedSeconds: number
  puttEventIds: string[]
  selectedPuttIndex: number
  startTraining: () => Promise<void>
  stopTraining: () => Promise<StoredTrainingSession | null>
  selectPreviousPutt: () => void
  selectNextPutt: () => void
  canGoPrevious: boolean
  canGoNext: boolean
  setSelectedPuttIndex: (index: number) => void
  syncError: string | null
}

const TrainingContext = createContext<TrainingContextType | undefined>(undefined)

function buildSessionId(userId?: string): string {
  const base = userId ? userId.slice(0, 8) : 'anon'
  const randomSuffix = Math.random().toString(36).slice(2, 8)
  return `session-${Date.now()}-${base}-${randomSuffix}`
}

export function TrainingProvider({ children }: { children: React.ReactNode }) {
  const { session } = useAuth()
  const [isTraining, setIsTraining] = useState(false)
  const [startedAt, setStartedAt] = useState<string | null>(null)
  const [elapsedSeconds, setElapsedSeconds] = useState(0)
  const [puttEventIds, setPuttEventIds] = useState<string[]>([])
  const [selectedPuttIndex, setSelectedPuttIndex] = useState(0)
  const [syncError, setSyncError] = useState<string | null>(null)

  const knownEventIdsRef = useRef<Set<string>>(new Set())

  useEffect(() => {
    if (!isTraining || !startedAt) {
      return
    }

    const intervalId = setInterval(() => {
      const duration = Math.floor((Date.now() - new Date(startedAt).getTime()) / 1000)
      setElapsedSeconds(Math.max(duration, 0))
    }, 1000)

    return () => clearInterval(intervalId)
  }, [isTraining, startedAt])

  useEffect(() => {
    if (!isTraining) {
      return
    }

    let cancelled = false

    const pollForNewPutts = async () => {
      try {
        const events = await loadStoredEvents()
        if (cancelled) return

        const newEventIds: string[] = []
        for (const event of events) {
          if (!knownEventIdsRef.current.has(event.id)) {
            knownEventIdsRef.current.add(event.id)
            newEventIds.push(event.id)
          }
        }

        if (newEventIds.length > 0) {
          setPuttEventIds((previous) => {
            const merged = [...previous, ...newEventIds]
            setSelectedPuttIndex(Math.max(0, merged.length - 1))
            return merged
          })
        }
      } catch {
        // Keep loop alive even if storage read fails once.
      }
    }

    void pollForNewPutts()
    const intervalId = setInterval(() => {
      void pollForNewPutts()
    }, 1200)

    return () => {
      cancelled = true
      clearInterval(intervalId)
    }
  }, [isTraining])

  const startTraining = async () => {
    if (isTraining) return

    const events = await loadStoredEvents()
    knownEventIdsRef.current = new Set(events.map((event) => event.id))

    const nowIso = new Date().toISOString()
    setSyncError(null)
    setStartedAt(nowIso)
    setElapsedSeconds(0)
    setPuttEventIds([])
    setSelectedPuttIndex(0)
    setIsTraining(true)
  }

  const stopTraining = async (): Promise<StoredTrainingSession | null> => {
    if (!isTraining || !startedAt) {
      return null
    }

    let finalPuttEventIds = puttEventIds
    try {
      const events = await loadStoredEvents()
      const extraIds = events
        .filter((event) => !knownEventIdsRef.current.has(event.id))
        .map((event) => event.id)

      if (extraIds.length > 0) {
        finalPuttEventIds = [...puttEventIds, ...extraIds]
        for (const eventId of extraIds) {
          knownEventIdsRef.current.add(eventId)
        }
      }
    } catch {
      // Keep stopping flow resilient if storage read fails.
    }

    const endedAt = new Date().toISOString()
    const durationSeconds = Math.max(
      1,
      Math.floor((new Date(endedAt).getTime() - new Date(startedAt).getTime()) / 1000)
    )

    const trainingSession: StoredTrainingSession = {
      id: buildSessionId(session?.user.id),
      userId: session?.user.id ?? null,
      startedAt,
      endedAt,
      durationSeconds,
      puttEventIds: finalPuttEventIds,
      puttsCount: finalPuttEventIds.length,
      createdAt: new Date().toISOString(),
    }

    await appendStoredTrainingSession(trainingSession)

    try {
      await syncTrainingSessionToSupabase(trainingSession)
      setSyncError(null)
    } catch (error) {
      const message =
        error instanceof Error
          ? error.message
          : 'Kunne ikke synkronisere session til Supabase'
      setSyncError(message)
    }

    setIsTraining(false)
    setStartedAt(null)
    setElapsedSeconds(0)
    setPuttEventIds([])
    setSelectedPuttIndex(0)
    knownEventIdsRef.current = new Set()

    return trainingSession
  }

  const selectPreviousPutt = () => {
    setSelectedPuttIndex((index) => Math.max(0, index - 1))
  }

  const selectNextPutt = () => {
    const maxIndex = Math.max(0, puttEventIds.length - 1)
    setSelectedPuttIndex((index) => Math.min(maxIndex, index + 1))
  }

  const canGoPrevious = selectedPuttIndex > 0
  const canGoNext = selectedPuttIndex < Math.max(0, puttEventIds.length - 1)

  const value = useMemo(
    () => ({
      isTraining,
      startedAt,
      elapsedSeconds,
      puttEventIds,
      selectedPuttIndex,
      startTraining,
      stopTraining,
      selectPreviousPutt,
      selectNextPutt,
      canGoPrevious,
      canGoNext,
      setSelectedPuttIndex,
      syncError,
    }),
    [
      isTraining,
      startedAt,
      elapsedSeconds,
      puttEventIds,
      selectedPuttIndex,
      canGoPrevious,
      canGoNext,
      syncError,
    ]
  )

  return <TrainingContext.Provider value={value}>{children}</TrainingContext.Provider>
}

export function useTraining() {
  const ctx = useContext(TrainingContext)
  if (!ctx) throw new Error('useTraining must be used inside TrainingProvider')
  return ctx
}
