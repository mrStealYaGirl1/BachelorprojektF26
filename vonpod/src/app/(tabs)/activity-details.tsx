import React, { useEffect, useMemo, useState } from 'react'
import {
  ActivityIndicator,
  FlatList,
  SafeAreaView,
  ScrollView,
  StyleSheet,
  Text,
  View,
} from 'react-native'
import { useLocalSearchParams } from 'expo-router'
import type { ImuSample } from '../../lib/imuPackets'
import { loadStoredEvents, type StoredImuEvent } from '../../lib/imuStorage'

const CHART_HEIGHT = 200
const BAR_WIDTH = 3

function GzChart({ samples }: { samples: ImuSample[] }) {
  const gzValues = useMemo(() => samples.map((sample) => sample.gz), [samples])

  const { min, max, bars } = useMemo(() => {
    if (gzValues.length === 0) {
      return { min: 0, max: 0, bars: [] as number[] }
    }

    const minValue = Math.min(...gzValues)
    const maxValue = Math.max(...gzValues)
    const range = Math.max(1, maxValue - minValue)

    const normalizedBars = gzValues.map((value) => {
      const normalized = (value - minValue) / range
      return Math.max(2, Math.round(normalized * (CHART_HEIGHT - 8)))
    })

    return {
      min: minValue,
      max: maxValue,
      bars: normalizedBars,
    }
  }, [gzValues])

  if (samples.length === 0) {
    return <Text style={styles.emptyText}>Ingen samples at plotte.</Text>
  }

  return (
    <View>
      <Text style={styles.chartMeta}>gz min: {min}</Text>
      <Text style={styles.chartMeta}>gz max: {max}</Text>

      <ScrollView horizontal showsHorizontalScrollIndicator>
        <View style={styles.chartRow}>
          {bars.map((height, index) => (
            <View key={`bar-${index}`} style={styles.barWrap}>
              <View style={[styles.bar, { height }]} />
            </View>
          ))}
        </View>
      </ScrollView>
    </View>
  )
}

const ActivityDetails = () => {
  const params = useLocalSearchParams<{ id?: string }>()
  const [event, setEvent] = useState<StoredImuEvent | null>(null)
  const [isLoading, setIsLoading] = useState(true)
  const [errorText, setErrorText] = useState<string | null>(null)

  useEffect(() => {
    const loadEvent = async () => {
      setIsLoading(true)
      setErrorText(null)

      try {
        const events = await loadStoredEvents()
        const found = events.find((entry) => entry.id === params.id)
        setEvent(found ?? null)
      } catch (error) {
        const message = error instanceof Error ? error.message : 'Kunne ikke indlaese event'
        setErrorText(message)
      } finally {
        setIsLoading(false)
      }
    }

    void loadEvent()
  }, [params.id])

  if (isLoading) {
    return (
      <SafeAreaView style={styles.container}>
        <ActivityIndicator />
      </SafeAreaView>
    )
  }

  if (!event) {
    return (
      <SafeAreaView style={styles.container}>
        <Text style={styles.title}>Event ikke fundet</Text>
        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
      </SafeAreaView>
    )
  }

  return (
    <SafeAreaView style={styles.container}>
      <FlatList
        data={event.samples}
        keyExtractor={(item, index) => `${item.seq}-${index}`}
        ListHeaderComponent={
          <View>
            <Text style={styles.title}>Event {event.eventId}</Text>
            <Text style={styles.subTitle}>Tidspunkt: {new Date(event.savedAt).toLocaleString()}</Text>
            <Text style={styles.subTitle}>Samples: {event.sampleCount}</Text>
            <Text style={styles.sectionTitle}>gz graf</Text>
            <GzChart samples={event.samples} />
            <Text style={styles.sectionTitle}>Sample-liste</Text>
          </View>
        }
        renderItem={({ item, index }) => (
          <View style={styles.sampleRow}>
            <Text style={styles.sampleText}>
              #{index + 1} | seq {item.seq} | t {item.tsMs} | gz {item.gz} | gx {item.gx} | gy {item.gy}
            </Text>
          </View>
        )}
        ListEmptyComponent={<Text style={styles.emptyText}>Ingen samples i eventet.</Text>}
        contentContainerStyle={styles.listContent}
      />

      {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    paddingHorizontal: 16,
    paddingTop: 18,
    backgroundColor: '#f4f5f6',
  },
  title: {
    fontSize: 26,
    fontWeight: '700',
    marginBottom: 8,
  },
  subTitle: {
    fontSize: 14,
    marginBottom: 4,
    opacity: 0.85,
  },
  sectionTitle: {
    marginTop: 14,
    marginBottom: 8,
    fontSize: 16,
    fontWeight: '700',
  },
  chartMeta: {
    fontSize: 12,
    opacity: 0.7,
  },
  chartRow: {
    marginTop: 8,
    height: CHART_HEIGHT,
    paddingVertical: 4,
    flexDirection: 'row',
    alignItems: 'flex-end',
    borderWidth: 1,
    borderColor: '#ccd0d4',
    borderRadius: 8,
    paddingHorizontal: 4,
    backgroundColor: '#ffffff',
  },
  barWrap: {
    width: BAR_WIDTH,
    marginRight: 1,
    height: CHART_HEIGHT - 8,
    justifyContent: 'flex-end',
  },
  bar: {
    width: BAR_WIDTH,
    backgroundColor: '#2f6f3f',
    borderTopLeftRadius: 2,
    borderTopRightRadius: 2,
  },
  sampleRow: {
    borderBottomWidth: 1,
    borderBottomColor: '#d9dde2',
    paddingVertical: 8,
  },
  sampleText: {
    fontSize: 12,
  },
  listContent: {
    paddingBottom: 30,
  },
  emptyText: {
    marginTop: 10,
    opacity: 0.8,
  },
  errorText: {
    marginTop: 10,
    color: '#b00020',
  },
})

export default ActivityDetails
