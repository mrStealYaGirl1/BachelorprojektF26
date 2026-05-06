import { useCallback, useMemo, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Pressable } from 'react-native';
import { useFocusEffect } from 'expo-router';
import FeatureCard from '../../components/cards/FeatureCard';
import { loadStoredEvents, type StoredImuEvent } from '../../lib/imuStorage';
import { getLatestValidTempo } from '../../features/puttMetrics/calculateTempo';
import { calculateFaceRotationFromEvent } from '../../features/puttMetrics/calculateFaceRotation';
import { calculateLieAngleImpactFromEvent } from '../../features/puttMetrics/calculateLieAngleImpact';
import { useTraining } from '../../providers/TrainingProvider';

function formatDuration(totalSeconds: number): string {
  const seconds = Math.max(0, Math.floor(totalSeconds));
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  const remainingSeconds = seconds % 60;

  if (hours > 0) {
    return `${hours}h ${minutes}m ${remainingSeconds}s`;
  }

  return `${minutes}m ${remainingSeconds}s`;
}

export default function StartTrainingScreen() {
  const {
    isTraining,
    elapsedSeconds,
    puttEventIds,
    selectedPuttIndex,
    canGoPrevious,
    canGoNext,
    selectPreviousPutt,
    selectNextPutt,
  } = useTraining();

  const [events, setEvents] = useState<StoredImuEvent[]>([]);

  const refreshEvents = useCallback(async () => {
    try {
      const parsed = await loadStoredEvents();
      setEvents(parsed);
    } catch {
      setEvents([]);
    }
  }, []);

  const visiblePuttCount = Math.max(1, puttEventIds.length);
  const activeDisplayIndex = Math.min(selectedPuttIndex, visiblePuttCount - 1);
  const activeEventId = puttEventIds[selectedPuttIndex];

  const selectedEvent = useMemo(
    () => events.find((event) => event.id === activeEventId) ?? null,
    [events, activeEventId]
  );

  const puttNumbers = useMemo(
    () => Array.from({ length: visiblePuttCount }, (_, index) => index + 1),
    [visiblePuttCount]
  );

  const selectedTempo = useMemo(
    () => (selectedEvent ? getLatestValidTempo([selectedEvent]) : null),
    [selectedEvent]
  );

  const selectedFaceRotation = useMemo(
    () => (selectedEvent ? calculateFaceRotationFromEvent(selectedEvent) : null),
    [selectedEvent]
  );

  const selectedLieAngleImpact = useMemo(
    () => (selectedEvent ? calculateLieAngleImpactFromEvent(selectedEvent) : null),
    [selectedEvent]
  );

  const forwardStrokeRotationDeg = useMemo(() => {
    if (!selectedFaceRotation) {
      return null;
    }

    return selectedFaceRotation.faceAngleAtImpactDeg - selectedFaceRotation.faceOpenAtTopDeg;
  }, [selectedFaceRotation]);

  const formatDegValue = (value: number | null): string => {
    if (value === null || !Number.isFinite(value)) {
      return '-.-';
    }

    return Math.abs(value).toFixed(1);
  };

  const directionLabel = (value: number | null, positiveLabel: string, negativeLabel: string): string => {
    if (value === null || !Number.isFinite(value)) {
      return 'deg';
    }

    return value >= 0 ? `deg · ${positiveLabel}` : `deg · ${negativeLabel}`;
  };

  useFocusEffect(
    useCallback(() => {
      void refreshEvents();

      const intervalId = setInterval(() => {
        void refreshEvents();
      }, 1000);

      return () => {
        clearInterval(intervalId);
      };
    }, [refreshEvents])
  );

  return (
    <View style={styles.screen}>
      <Text style={styles.logo}>vonpod</Text>

      <Text style={styles.sessionState}>{isTraining ? 'Session i gang' : 'Ingen aktiv session'}</Text>
      <Text style={styles.sessionTimer}>Timer: {formatDuration(elapsedSeconds)}</Text>

      <View style={styles.navigator}>
        <Pressable
          style={[styles.arrowButton, !canGoPrevious && styles.arrowButtonDisabled]}
          onPress={selectPreviousPutt}
          disabled={!canGoPrevious}
        >
          <Text style={styles.arrow}>‹</Text>
        </Pressable>

        <View style={styles.puttNumbers}>
          {puttNumbers.map((number, index) => (
            <Text key={`putt-${number}`} style={index === activeDisplayIndex ? styles.activePutt : styles.inactivePutt}>
              {number}
            </Text>
          ))}
        </View>

        <Pressable
          style={[styles.arrowButton, !canGoNext && styles.arrowButtonDisabled]}
          onPress={selectNextPutt}
          disabled={!canGoNext}
        >
          <Text style={styles.arrow}>›</Text>
        </Pressable>
      </View>

      <ScrollView
        contentContainerStyle={styles.scrollContent}
        showsVerticalScrollIndicator={false}
      >
        <View style={styles.grid}>
          <FeatureCard
            title="Tempo"
            value={selectedTempo?.ratioLabel ?? '-.-'}
            unitLabel="ratio"
          />
          <FeatureCard
            title="Backswing Length"
            value={selectedEvent ? String(selectedEvent.sampleCount) : '-.-'}
            unitLabel="cm"
          />
          <FeatureCard
            title="Back Stroke Rotation"
            value={formatDegValue(selectedFaceRotation?.faceOpenAtTopDeg ?? null)}
            unitLabel={directionLabel(selectedFaceRotation?.faceOpenAtTopDeg ?? null, 'open', 'closed')}
          />
          <FeatureCard
            title="Forward Stroke Rotation"
            value={formatDegValue(forwardStrokeRotationDeg)}
            unitLabel={directionLabel(forwardStrokeRotationDeg, 'open', 'closed')}
          />
          <FeatureCard
            title="Lie Angle Impact"
            value={formatDegValue(selectedLieAngleImpact?.impactAngleDeg ?? null)}
            unitLabel={selectedLieAngleImpact?.impactAngleUnitLabel ?? 'deg'}
          />
          <FeatureCard
            title="Face Angle at Impact"
            value={formatDegValue(selectedFaceRotation?.faceAngleAtImpactDeg ?? null)}
            unitLabel={directionLabel(selectedFaceRotation?.faceAngleAtImpactDeg ?? null, 'open', 'closed')}
          />
        </View>

        <Pressable style={styles.reorderButton}>
          <Text style={styles.reorderText}>Reorder</Text>
          <Text style={styles.reorderIcon}>▦</Text>
        </Pressable>
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  screen: {
    flex: 1,
    backgroundColor: '#F4F3F1',
    paddingTop: 54,
    paddingHorizontal: 10,
  },
  logo: {
    fontSize: 28,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: {width: -1, height: 1},
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#222',
    marginBottom: 4,
  },
  navigator: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  sessionState: {
    marginTop: 6,
    textAlign: 'center',
    fontSize: 14,
    color: '#4a4a4a',
    fontFamily: 'RethinkSans_500Medium',
  },
  sessionTimer: {
    marginTop: 2,
    marginBottom: 10,
    textAlign: 'center',
    fontSize: 16,
    color: '#1c1c1c',
    fontFamily: 'Montserrat_600SemiBold',
  },
  arrowButton: {
    width: 28,
    alignItems: 'center',
    justifyContent: 'center',
  },
  arrowButtonDisabled: {
    opacity: 0.35,
  },
  arrow: {
    fontSize: 34,
    color: '#111',
    fontWeight: '300',
    lineHeight: 34,
  },
  puttNumbers: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 18,
  },
  inactivePutt: {
    fontSize: 18,
    fontFamily: 'Montserrat_400Regular',
    color: '#C9C7C4',
    fontWeight: '400',
  },
  activePutt: {
    fontSize: 18,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#111',
    fontWeight: '600',
  },
  scrollContent: {
    paddingBottom: 140,
  },
  grid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  reorderButton: {
    alignSelf: 'center',
    marginTop: 6,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
    backgroundColor: '#ECE8E6',
    borderRadius: 999,
    paddingHorizontal: 18,
    paddingVertical: 8,
  },
  reorderText: {
    fontSize: 12,
    fontWeight: '500',
    color: '#111',
    fontFamily: 'RethinkSans_400Regular',
  },
  reorderIcon: {
    fontSize: 18,
    color: '#111',
  },
});