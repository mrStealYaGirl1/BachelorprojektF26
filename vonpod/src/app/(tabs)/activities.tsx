import React, { useCallback, useEffect, useState } from 'react'
import { Button, Pressable, SafeAreaView, ScrollView, StyleSheet, Text, View } from 'react-native'
import { useRouter } from 'expo-router'
import {
  clearStoredEvents,
  loadStoredEvents,
  type StoredImuEvent,
} from '../../lib/imuStorage'

const Activities = () => {
  const router = useRouter()
  const [events, setEvents] = useState<StoredImuEvent[]>([])
  const [errorText, setErrorText] = useState<string | null>(null)
  const [isLoading, setIsLoading] = useState(false)

  const loadEvents = useCallback(async () => {
    setIsLoading(true)
    setErrorText(null)

    try {
      const parsed = await loadStoredEvents()
      const sorted = [...parsed].sort(
        (a, b) => new Date(b.savedAt).getTime() - new Date(a.savedAt).getTime()
      )
      setEvents(sorted)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke indlaese events'
      setErrorText(message)
    } finally {
      setIsLoading(false)
    }
  }, [])

  const clearEvents = useCallback(async () => {
    setIsLoading(true)
    setErrorText(null)

    try {
      await clearStoredEvents()
      setEvents([])
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke slette events'
      setErrorText(message)
    } finally {
      setIsLoading(false)
    }
  }, [])

  useEffect(() => {
    void loadEvents()
  }, [loadEvents])

  return (
    <SafeAreaView style={styles.container}>
      <Text style={styles.title}>Activities</Text>
      <Text style={styles.subtitle}>Gemte BLE-events: {events.length}</Text>

      <View style={styles.buttonRow}>
        <Button title={isLoading ? 'Indlaeser...' : 'Opdater liste'} onPress={loadEvents} />
      </View>

      <View style={styles.buttonRow}>
        <Button title='Slet alle events' onPress={clearEvents} color='#b00020' />
      </View>

      {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}

      <ScrollView contentContainerStyle={styles.listContent}>
        {events.length === 0 ? (
          <Text style={styles.emptyText}>Ingen gemte events endnu.</Text>
        ) : (
          events.map((event) => (
            <Pressable
              key={event.id}
              style={styles.card}
              onPress={() =>
                router.push({
                  pathname: '/activity-details',
                  params: { id: event.id },
                })
              }
            >
              <Text style={styles.cardTitle}>Event {event.eventId}</Text>
              <Text style={styles.cardText}>Gemte samples: {event.sampleCount}</Text>
              <Text style={styles.cardText}>Tidspunkt: {new Date(event.savedAt).toLocaleString()}</Text>
              <Text style={styles.cardText}>
                META: {event.meta ? `swing ${event.meta.swingId}, ${event.meta.sampleRateHz} Hz` : 'ingen'}
              </Text>

              {event.samples.length > 0 && (
                <Text style={styles.cardText}>
                  Seneste: seq {event.samples[event.samples.length - 1].seq}, gyro{' '}
                  {event.samples[event.samples.length - 1].gyroDps.toFixed(1)} dps
                </Text>
              )}

              <Text style={styles.openText}>Tryk for at aabne graf og sample-liste</Text>
            </Pressable>
          ))
        )}
      </ScrollView>
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingHorizontal: 16,
    paddingTop: 24,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
    marginBottom: 8,
  },
  subtitle: {
    fontSize: 16,
    marginBottom: 16,
    opacity: 0.8,
  },
  buttonRow: {
    marginBottom: 10,
  },
  listContent: {
    paddingBottom: 24,
  },
  emptyText: {
    marginTop: 12,
    fontSize: 14,
    opacity: 0.8,
  },
  card: {
    borderWidth: 1,
    borderColor: '#d2d2d2',
    borderRadius: 10,
    padding: 12,
    marginTop: 10,
    backgroundColor: '#ffffff',
  },
  cardTitle: {
    fontSize: 16,
    fontWeight: '700',
    marginBottom: 6,
  },
  cardText: {
    fontSize: 13,
    marginBottom: 3,
  },
  openText: {
    marginTop: 6,
    fontSize: 12,
    color: '#2f6f3f',
    fontWeight: '600',
  },
  errorText: {
    color: '#b00020',
    marginBottom: 8,
  },
})

export default Activities