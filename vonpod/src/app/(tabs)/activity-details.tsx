import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  ActivityIndicator,
  Alert,
  Animated,
  Easing,
  FlatList,
  LayoutAnimation,
  Platform,
  Pressable,
  SafeAreaView,
  ScrollView,
  StyleSheet,
  Text,
  UIManager,
  View,
} from 'react-native'
import { useLocalSearchParams, useRouter } from 'expo-router'
import Ionicons from '@expo/vector-icons/Ionicons'
import { useFocusEffect } from '@react-navigation/native'
import type { ImuSample, MetaPacket } from '../../lib/imuPackets'
import { loadStoredEvents, type StoredImuEvent } from '../../lib/imuStorage'
import { loadStoredTrainingSessions, type StoredTrainingSession } from '../../lib/trainingStorage'
import { calculateTempoFromMeta } from '../../features/puttMetrics/calculateTempo'
import { calculateFaceRotationFromEvent } from '../../features/puttMetrics/calculateFaceRotation'
import { calculateLieAngleImpactFromEvent } from '../../features/puttMetrics/calculateLieAngleImpact'
import { createPuttExportZip, sharePuttExportZip } from '../../features/export/exportPuttCsv'
import PuttGyroZChart from '../../components/charts/PuttGyroZChart'
import { BlurView } from 'expo-blur'
import { LinearGradient } from 'expo-linear-gradient'
import MaskedView from '@react-native-masked-view/masked-view'

const CHART_HEIGHT = 200
const POINT_STEP_X = 2
const POINT_SIZE = 2
const CHART_HORIZONTAL_PADDING = 4
const CHART_VERTICAL_PADDING = 4
const TAB_SWITCH_PADDING = 4
const TAB_SWITCH_GAP = 0

type SessionDetailsTab = 'summary' | 'putts'

type ChartPoint = {
  x: number
  y: number
}

type PhaseMarker = {
  key: 'backswing' | 'forward' | 'impact'
  label: string
  color: string
  left: number
}

type Segment = {
  key: string
  left: number
  top: number
  width: number
  angleDeg: string
}

function findNearestSampleIndex(samples: ImuSample[], targetTsMs: number): number {
  let bestIndex = 0
  let bestDistance = Number.POSITIVE_INFINITY

  for (let i = 0; i < samples.length; i += 1) {
    const distance = Math.abs(samples[i].tsMs - targetTsMs)
    if (distance < bestDistance) {
      bestDistance = distance
      bestIndex = i
    }
  }

  return bestIndex
}

function buildPhaseMarkers(samples: ImuSample[], meta?: MetaPacket): PhaseMarker[] {
  if (!meta || samples.length === 0) {
    return []
  }

  const eventStartSampleTsMs = samples[0].tsMs

  const markers = [
    {
      key: 'backswing' as const,
      label: 'BS',
      color: '#1976d2',
      targetTsMs: (meta.backswingStartUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
    {
      key: 'forward' as const,
      label: 'FS',
      color: '#ef6c00',
      targetTsMs: (meta.forwardStartUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
    {
      key: 'impact' as const,
      label: 'IMP',
      color: '#c62828',
      targetTsMs: (meta.impactUs - meta.eventStartUs) / 1000 + eventStartSampleTsMs,
    },
  ]

  return markers
    .filter((marker) => Number.isFinite(marker.targetTsMs))
    .map((marker) => {
      const index = findNearestSampleIndex(samples, marker.targetTsMs)
      const left = CHART_HORIZONTAL_PADDING + index * POINT_STEP_X

      return {
        key: marker.key,
        label: marker.label,
        color: marker.color,
        left,
      }
    })
}

function GzChart({ samples, meta }: { samples: ImuSample[]; meta?: MetaPacket }) {
  const gzValues = useMemo(() => samples.map((sample) => sample.gz), [samples])
  const markers = useMemo(() => buildPhaseMarkers(samples, meta), [samples, meta])

  const { min, max, points, segments, zeroLineTop, chartWidth } = useMemo(() => {
    if (gzValues.length === 0) {
      return {
        min: 0,
        max: 0,
        points: [] as ChartPoint[],
        segments: [] as Segment[],
        zeroLineTop: null as number | null,
        chartWidth: CHART_HORIZONTAL_PADDING * 2,
      }
    }

    const minValue = Math.min(...gzValues)
    const maxValue = Math.max(...gzValues)
    const range = Math.max(1, maxValue - minValue)
    const drawableHeight = CHART_HEIGHT - CHART_VERTICAL_PADDING * 2
    const mappedPoints = gzValues.map((value, index) => {
      const normalized = (value - minValue) / range
      const y = CHART_VERTICAL_PADDING + (1 - normalized) * drawableHeight
      const x = CHART_HORIZONTAL_PADDING + index * POINT_STEP_X

      return { x, y }
    })

    const mappedSegments: Segment[] = []
    for (let i = 0; i < mappedPoints.length - 1; i += 1) {
      const start = mappedPoints[i]
      const end = mappedPoints[i + 1]
      const dx = end.x - start.x
      const dy = end.y - start.y
      const width = Math.sqrt(dx * dx + dy * dy)
      const angleDeg = `${(Math.atan2(dy, dx) * 180) / Math.PI}deg`

      mappedSegments.push({
        key: `seg-${i}`,
        left: start.x,
        top: start.y,
        width,
        angleDeg,
      })
    }

    const zeroLineTop =
      minValue <= 0 && maxValue >= 0
        ? CHART_VERTICAL_PADDING + ((maxValue - 0) / range) * drawableHeight
        : null

    const width = CHART_HORIZONTAL_PADDING * 2 + Math.max(1, gzValues.length - 1) * POINT_STEP_X

    return {
      min: minValue,
      max: maxValue,
      points: mappedPoints,
      segments: mappedSegments,
      zeroLineTop,
      chartWidth: width,
    }
  }, [gzValues])

  if (samples.length === 0) {
    return <Text style={styles.emptyText}>Ingen samples at plotte.</Text>
  }

  return (
    <View>
      <Text style={styles.chartMeta}>gz min: {min}</Text>
      <Text style={styles.chartMeta}>gz max: {max}</Text>
      <Text style={styles.chartMeta}>BS=backswing start, FS=forward start, IMP=impact</Text>

      <ScrollView horizontal showsHorizontalScrollIndicator>
        <View style={[styles.chartRow, { width: chartWidth }]}>
          {zeroLineTop !== null && <View style={[styles.zeroLine, { top: zeroLineTop }]} />}

          {markers.map((marker) => (
            <View key={marker.key} style={[styles.markerWrap, { left: marker.left }]}>
              <Text style={[styles.markerLabel, { color: marker.color }]}>{marker.label}</Text>
              <View style={[styles.markerLine, { backgroundColor: marker.color }]} />
            </View>
          ))}

          {segments.map((segment) => (
            <View
              key={segment.key}
              style={[
                styles.lineSegment,
                {
                  left: segment.left,
                  top: segment.top,
                  width: segment.width,
                  transform: [{ rotate: segment.angleDeg }],
                },
              ]}
            />
          ))}

          {points.map((point, index) => (
            <View
              key={`pt-${index}`}
              style={[
                styles.point,
                {
                  left: point.x - POINT_SIZE / 2,
                  top: point.y - POINT_SIZE / 2,
                },
              ]}
            />
          ))}
        </View>
      </ScrollView>
    </View>
  )
}

const ActivityDetails = () => {
  const router = useRouter()
  const params = useLocalSearchParams<{ id?: string; sessionId?: string }>()
  const [event, setEvent] = useState<StoredImuEvent | null>(null)
  const [trainingSession, setTrainingSession] = useState<StoredTrainingSession | null>(null)
  const [sessionEvents, setSessionEvents] = useState<StoredImuEvent[]>([])
  const [activeTab, setActiveTab] = useState<SessionDetailsTab>('summary')
  const [preExportTab, setPreExportTab] = useState<SessionDetailsTab>('summary')
  const [expandedPuttId, setExpandedPuttId] = useState<string | null>(null)
  const [isExportMode, setIsExportMode] = useState(false)
  const [selectedExportIds, setSelectedExportIds] = useState<string[]>([])
  const [isExporting, setIsExporting] = useState(false)
  const [isCompareToSession, setIsCompareToSession] = useState(false)
  const [isLoading, setIsLoading] = useState(true)
  const [errorText, setErrorText] = useState<string | null>(null)
  const [tabSwitchWidth, setTabSwitchWidth] = useState(0)
  const tabIndicatorValue = useRef(new Animated.Value(activeTab === 'summary' ? 0 : 1)).current
  const tabInnerWidth = Math.max(0, tabSwitchWidth - TAB_SWITCH_PADDING * 2)
  const tabHalfWidth = tabInnerWidth / 2

  /*
  const tabButtonWidth = useMemo(() => {
    if (tabSwitchWidth <= 0) return 0
    return (tabSwitchWidth - TAB_SWITCH_PADDING * 2) / 2
  }, [tabSwitchWidth])

  const tabIndicatorTranslateX = useMemo(
    () =>
      tabIndicatorValue.interpolate({
        inputRange: [0, 1],
        outputRange: [0, tabButtonWidth],
      }),
    [tabButtonWidth, tabIndicatorValue]
  ) */

  const tabIndicatorTranslateX = tabIndicatorValue.interpolate({
    inputRange: [0, 1],
    outputRange: [0, tabHalfWidth],
  })

  useEffect(() => {
    if (Platform.OS === 'android' && UIManager.setLayoutAnimationEnabledExperimental) {
      UIManager.setLayoutAnimationEnabledExperimental(true)
    }
  }, [])

  const formatSessionDuration = (totalSeconds: number): string => {
    const seconds = Math.max(0, Math.floor(totalSeconds))
    const minutes = Math.floor(seconds / 60)
    const remainingSeconds = seconds % 60
    return `${minutes}m ${remainingSeconds.toString().padStart(2, '0')}s`
  }

  const getDefaultActivityName = (startedAt: string): string => {
    const date = new Date(startedAt)
    const dateLabel = date.toLocaleDateString('en-GB', {
      day: '2-digit',
      month: 'short',
      year: 'numeric',
    })
    return `Training session ${dateLabel}`
  }

  const getSessionDateLabel = (startedAt: string): string => {
    const date = new Date(startedAt)
    return date.toLocaleDateString('en-US', {
      month: 'short',
      day: 'numeric',
      year: 'numeric',
    })
  }

  const buildPuttMetrics = (puttEvent: StoredImuEvent) => {
    const tempo = puttEvent.meta ? calculateTempoFromMeta(puttEvent.meta) : null
    const faceRotation = calculateFaceRotationFromEvent(puttEvent)
    const lieAngleImpact = calculateLieAngleImpactFromEvent(puttEvent)
    const tempoLabel = tempo?.ratioLabel ?? '-.-:1'

    const backStrokeRotationDeg = faceRotation ? faceRotation.faceOpenAtTopDeg : null
    const forwardStrokeRotationDeg = faceRotation
      ? faceRotation.faceAngleAtImpactDeg - faceRotation.faceOpenAtTopDeg
      : null
    const impactAngleDeg = lieAngleImpact ? lieAngleImpact.impactAngleDeg : null

    const formatDegValue = (value: number | null): string => {
      if (value === null || !Number.isFinite(value)) {
        return '-.-'
      }

      return Math.abs(value).toFixed(1)
    }

    const formatRotationDirection = (value: number | null): string => {
      if (value === null || !Number.isFinite(value)) {
        return 'deg'
      }

      return value >= 0 ? 'deg • open' : 'deg • closed'
    }

    return {
      tempoLabel,
      impactAngle: formatDegValue(impactAngleDeg),
      impactAngleUnit: lieAngleImpact?.impactAngleUnitLabel ?? 'deg',
      backStrokeRotation: formatDegValue(backStrokeRotationDeg),
      backStrokeRotationUnit: formatRotationDirection(backStrokeRotationDeg),
      forwardStrokeRotation: formatDegValue(forwardStrokeRotationDeg),
      forwardStrokeRotationUnit: formatRotationDirection(forwardStrokeRotationDeg),
    }
  }

  const getPuttWindowDurationMs = (puttEvent: StoredImuEvent): number => {
    if (!puttEvent.meta || puttEvent.samples.length === 0) {
      return 0
    }

    const eventStartSampleTsMs = puttEvent.samples[0].tsMs
    const startTsMs =
      (puttEvent.meta.backswingStartUs - puttEvent.meta.eventStartUs) / 1000 + eventStartSampleTsMs
    const followWindowEndUs = puttEvent.meta.followStartUs + 200000
    const endTsMs =
      (followWindowEndUs - puttEvent.meta.eventStartUs) / 1000 + eventStartSampleTsMs

    return Math.max(0, endTsMs - startTsMs)
  }

  const loadData = useCallback(async () => {
    setIsLoading(true)
    setErrorText(null)

    try {
      const events = await loadStoredEvents()

      if (params.sessionId) {
        const sessions = await loadStoredTrainingSessions()
        const foundSession = sessions.find((entry) => entry.id === params.sessionId)

        setTrainingSession(foundSession ?? null)
        setEvent(null)

        if (foundSession) {
          const mappedSessionEvents = foundSession.puttEventIds
            .map((puttEventId) => events.find((entry) => entry.id === puttEventId))
            .filter((entry): entry is StoredImuEvent => !!entry)

          setSessionEvents(mappedSessionEvents)
        } else {
          setSessionEvents([])
        }

        return
      }

      const found = events.find((entry) => entry.id === params.id)
      setEvent(found ?? null)
      setTrainingSession(null)
      setSessionEvents([])
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke indlaese event'
      setErrorText(message)
    } finally {
      setIsLoading(false)
    }
  }, [params.id, params.sessionId])

  useEffect(() => {
    void loadData()
  }, [loadData])

  useFocusEffect(
    useCallback(() => {
      void loadData()
    }, [loadData])
  )

  useEffect(() => {
    setActiveTab('summary')
    setExpandedPuttId(null)
    setIsExportMode(false)
    setSelectedExportIds([])
    setIsExporting(false)
    setIsCompareToSession(false)
  }, [params.sessionId])

  useEffect(() => {
    setIsCompareToSession(false)
  }, [expandedPuttId, isExportMode])

  useEffect(() => {
    Animated.timing(tabIndicatorValue, {
      toValue: activeTab === 'summary' ? 0 : 1,
      duration: 220,
      easing: Easing.out(Easing.cubic),
      useNativeDriver: true,
    }).start()
  }, [activeTab, tabIndicatorValue])

  const isAllPuttsSelected = sessionEvents.length > 0 && selectedExportIds.length === sessionEvents.length

  const sessionDurationById = useMemo(() => {
    const map = new Map<string, number>()
    sessionEvents.forEach((event) => {
      map.set(event.id, getPuttWindowDurationMs(event))
    })
    return map
  }, [sessionEvents])

  const toggleExportMode = useCallback(() => {
    setErrorText(null)

    if (isExportMode) {
      setIsExportMode(false)
      setSelectedExportIds([])
      setIsExporting(false)
      setActiveTab(preExportTab)
      return
    }

    setPreExportTab(activeTab)
    setActiveTab('putts')
    setExpandedPuttId(null)
    setIsExportMode(true)
    setSelectedExportIds([])
  }, [activeTab, isExportMode, preExportTab])

  const togglePuttSelection = useCallback((puttId: string) => {
    setSelectedExportIds((previousIds) =>
      previousIds.includes(puttId)
        ? previousIds.filter((id) => id !== puttId)
        : [...previousIds, puttId]
    )
  }, [])

  const toggleSelectAllPutts = useCallback(() => {
    if (sessionEvents.length === 0) {
      return
    }

    setSelectedExportIds((previousIds) => {
      if (previousIds.length === sessionEvents.length) {
        return []
      }

      return sessionEvents.map((event) => event.id)
    })
  }, [sessionEvents])

  const handleExportSelectedPutts = useCallback(
    async (sessionTitle: string) => {
      if (selectedExportIds.length === 0) {
        Alert.alert('Ingen putts valgt', 'Vaelg mindst en putt for at eksportere.')
        return
      }

      setIsExporting(true)
      setErrorText(null)

      try {
        const selectedPutts = sessionEvents.filter((event) => selectedExportIds.includes(event.id))
        const zipUri = await createPuttExportZip(sessionTitle, selectedPutts)
        await sharePuttExportZip(zipUri)
      } catch (error) {
        const message = error instanceof Error ? error.message : 'Eksport fejlede'
        setErrorText(message)
      } finally {
        setIsExporting(false)
      }
    },
    [selectedExportIds, sessionEvents]
  )

  if (isLoading) {
    return (
      <SafeAreaView style={styles.container}>
        <ActivityIndicator />
      </SafeAreaView>
    )
  }

  if (!event && !trainingSession) {
    return (
      <SafeAreaView style={styles.container}>
        <Text style={styles.title}>Data ikke fundet</Text>
        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
      </SafeAreaView>
    )
  }

  if (trainingSession) {
    const sessionTitle =
      trainingSession.activityName?.trim() || getDefaultActivityName(trainingSession.startedAt)
    const sessionMetaLabel = `${getSessionDateLabel(trainingSession.startedAt)}  •  ${formatSessionDuration(trainingSession.durationSeconds)}  •  ${trainingSession.puttsCount} putts`

    return (
      <SafeAreaView style={styles.container}>
        <View style={styles.topBar}>
          <Pressable onPress={() => router.push('/activities')} style={styles.backButton}>
            <Ionicons name='chevron-back' size={24} color='#7b7b7b' />
          </Pressable>
          <Text style={styles.logo}>vonpod</Text>
          <View style={styles.backButtonPlaceholder} />
        </View>
        <View style={styles.pageTitleRow}>
          <Text
            style={styles.pageTitleInline}
            numberOfLines={1}
            adjustsFontSizeToFit
            minimumFontScale={0.75}
          >
            {sessionTitle}
          </Text>
          <Pressable
            onPress={toggleExportMode}
            style={[styles.exportToggleButton, isExportMode && styles.exportToggleButtonActive]}
          >
            <Ionicons
              name={isExportMode ? 'close' : 'share-outline'}
              size={14}
              color={isExportMode ? '#ffffff' : '#fff'}
            />
            <Text style={[styles.exportToggleButtonText, isExportMode && styles.exportToggleButtonTextActive]}>
              {isExportMode ? 'Cancel' : 'Export'}
            </Text>
          </Pressable>
        </View>

        <View style={styles.sessionHeaderBlock}>
          <Text style={styles.subTitle}>{sessionMetaLabel}</Text>

          {isExportMode ? (
            <View style={styles.exportHeaderRow}>
              <Text style={styles.exportHeaderTitle}>Select putts to export</Text>
              <Pressable style={styles.exportSelectAllButton} onPress={toggleSelectAllPutts}>
                <Text style={styles.exportSelectAllText}>Select all</Text>
                <View
                  style={[
                    styles.checkboxBox,
                    isAllPuttsSelected
                      ? styles.checkboxBoxSelected
                      : styles.checkboxBoxUnselected,
                  ]}
                >
                  {isAllPuttsSelected ? (
                    <Ionicons name="checkmark" size={18} color="#5f8457" />
                  ) : null}
                </View>
              </Pressable>
            </View>
          ) : (
            <View
              style={styles.tabSwitchWrap}
              onLayout={(event) => setTabSwitchWidth(event.nativeEvent.layout.width)}
            >
              <Animated.View
                pointerEvents="none"
                style={[
                  styles.tabSwitchActivePill,
                  {
                    width: tabHalfWidth,
                    transform: [{ translateX: tabIndicatorTranslateX }],
                  },
                ]}
              />

              <Pressable
                style={styles.tabSwitchButton}
                onPress={() => setActiveTab('summary')}
              >
                <Text
                  style={[
                    styles.tabSwitchText,
                    { color: activeTab === 'summary' ? '#ffffff' : '#2c2c2c' },
                  ]}
                  numberOfLines={1}
                >
                  Session Summary
                </Text>
              </Pressable>

              <Pressable
                style={styles.tabSwitchButton}
                onPress={() => setActiveTab('putts')}
              >
                <Text
                  style={[
                    styles.tabSwitchText,
                    { color: activeTab === 'putts' ? '#ffffff' : '#2c2c2c' },
                  ]}
                  numberOfLines={1}
                >
                  Putts
                </Text>
              </Pressable>
            </View>
          )}
        </View>

        {activeTab === 'summary' ? (
          <ScrollView contentContainerStyle={styles.listContent}>
            <View style={styles.summaryCard}>
              <Text style={styles.summaryCardTitle}>Rhythm</Text>
              <View style={styles.graphPlaceholder}>
                <Text style={styles.graphPlaceholderText}>Graph placeholder</Text>
            </View>
            </View>

            <View style={styles.summaryCard}>
              <Text style={styles.summaryCardTitle}>Backswing Length</Text>
              <View style={styles.graphPlaceholder}>
                <Text style={styles.graphPlaceholderText}>Graph placeholder</Text>
              </View>
            </View>
          </ScrollView>
        ) : (
          <FlatList
            data={sessionEvents}
            keyExtractor={(item) => item.id}
            contentContainerStyle={[
              styles.listContent,
              isExportMode && styles.listContentWithExportBar,
            ]}
            renderItem={({ item, index }) => {
              const metrics = buildPuttMetrics(item)
              const isExpanded = !isExportMode && expandedPuttId === item.id
              const isSelectedForExport = selectedExportIds.includes(item.id)
              const compareEnabled = isExpanded && isCompareToSession
              const compareTargets = compareEnabled
                ? sessionEvents.filter((event) => event.id !== item.id)
                : []

              const sharedAxisDurationMs = compareEnabled
                ? compareTargets.reduce((maxDuration, event) => {
                    const duration = sessionDurationById.get(event.id) ?? 0
                    return Math.max(maxDuration, duration)
                  }, sessionDurationById.get(item.id) ?? 0)
                : undefined

              const compareSamples = compareEnabled
                ? compareTargets.map((event) => ({ samples: event.samples, meta: event.meta }))
                : undefined

              return (
                <View style={[styles.puttCard, isExpanded && styles.puttCardExpanded]}>
                  <Pressable
                    style={styles.puttRowTop}
                    onPress={() => {
                      if (isExportMode) {
                        togglePuttSelection(item.id)
                        return
                      }

                      LayoutAnimation.configureNext(LayoutAnimation.Presets.easeInEaseOut)
                      setExpandedPuttId((prev) => (prev === item.id ? null : item.id))
                    }}
                  >
                    <View style={styles.puttNumberBadge}>
                      <Text style={styles.puttNumberText}>{index + 1}</Text>
                    </View>

                    <View style={styles.puttMetricsRow}>
                      <View style={styles.puttMetricCol}>
                        <Text style={styles.puttMetricLabel}>Tempo</Text>
                        <Text style={styles.puttMetricValue}>{metrics.tempoLabel}</Text>
                        <Text style={styles.puttMetricUnit}>ratio</Text>
                      </View>

                      <View style={styles.puttMetricCol}>
                        <Text style={styles.puttMetricLabel}>Imp. Angle</Text>
                        <Text style={styles.puttMetricValue}>{metrics.impactAngle}</Text>
                        <Text style={styles.puttMetricUnit}>{metrics.impactAngleUnit}</Text>
                      </View>

                      <View style={styles.puttMetricCol}>
                        <Text style={styles.puttMetricLabel}>BS Rot</Text>
                        <Text style={styles.puttMetricValue}>{metrics.backStrokeRotation}</Text>
                        <Text style={styles.puttMetricUnit}>{metrics.backStrokeRotationUnit}</Text>
                      </View>

                      <View style={styles.puttMetricCol}>
                        <Text style={styles.puttMetricLabel}>FS Rot</Text>
                        <Text style={styles.puttMetricValue}>{metrics.forwardStrokeRotation}</Text>
                        <Text style={styles.puttMetricUnit}>{metrics.forwardStrokeRotationUnit}</Text>
                      </View>
                    </View>

                    {isExportMode ? (
                      <View
                        style={[
                          styles.checkboxBox,
                          isSelectedForExport
                            ? styles.checkboxBoxSelected
                            : styles.checkboxBoxUnselected,
                        ]}
                      >
                        {isSelectedForExport ? (
                          <Ionicons name="checkmark" size={18} color="#5f8457" />
                        ) : null}
                      </View>
                    ) : (
                      <Ionicons
                        name={isExpanded ? 'chevron-up' : 'chevron-down'}
                        size={22}
                        color="#6f916a"
                        style={styles.chevronIcon}
                      />
                    )}
                  </Pressable>

                  {isExpanded && (
                    <View style={styles.puttExpandedBlock}>
                      <View style={styles.compareRow}>
                        <Text style={styles.compareText}>Compare to session</Text>
                        <Pressable
                          onPress={() => setIsCompareToSession((previous) => !previous)}
                          style={styles.compareToggle}
                        >
                          <View
                            style={[
                              styles.compareCheckboxBox,
                              isCompareToSession
                                ? styles.compareCheckboxBoxSelected
                                : styles.compareCheckboxBoxUnselected,
                            ]}
                          >
                            {isCompareToSession ? (
                              <Ionicons name="checkmark" size={12} color="#5f8457" />
                            ) : null}
                          </View>
                        </Pressable>
                      </View>

                      <PuttGyroZChart
                        samples={item.samples}
                        meta={item.meta}
                        compareSamples={compareSamples}
                        sharedAxisDurationMs={sharedAxisDurationMs}
                      />
                    </View>
                  )}
                </View>
              )
            }}
            ListEmptyComponent={<Text style={styles.emptyText}>Ingen putts registreret i denne session.</Text>}
          />
        )}

        {isExportMode && (
          <View pointerEvents="none" style={styles.bottomFadeBlurWrap}>
            <MaskedView
              style={StyleSheet.absoluteFill}
              maskElement={
                <LinearGradient
                  style={StyleSheet.absoluteFill}
                  colors={['transparent', 'rgba(0,0,0,0.65)', 'black']}
                  locations={[0, 0.45, 1]}
                />
              }
            >
              <BlurView
                intensity={80}
                tint="light"
                style={StyleSheet.absoluteFill}
              />
            </MaskedView>
          </View>
        )}

        {isExportMode && (
          <View style={styles.exportActionBarWrap}>
            <Pressable
              style={[
                styles.exportActionButton,
                (selectedExportIds.length === 0 || isExporting) &&
                  styles.exportActionButtonDisabled,
              ]}
              onPress={() => void handleExportSelectedPutts(sessionTitle)}
              disabled={selectedExportIds.length === 0 || isExporting}
            >
              <Text style={styles.exportActionButtonText}>
                {isExporting
                  ? 'Preparing export...'
                  : `Export selected putts (${selectedExportIds.length})`}
              </Text>
            </Pressable>
          </View>
        )}

        {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}
      </SafeAreaView>
    )
  }

  const singleEvent = event
  if (!singleEvent) {
    return null
  }

  return (
    <SafeAreaView style={styles.container}>
      <View style={styles.topBar}>
        <Pressable onPress={() => router.push('/activities')} style={styles.backButton}>
          <Ionicons name='chevron-back' size={24} color='#2c2c2c' />
        </Pressable>
        <Text style={styles.logo}>vonpod</Text>
        <View style={styles.backButtonPlaceholder} />
      </View>
      <Text style={styles.pageTitle}>Putt #{singleEvent.localEventId ?? '-'}</Text>
      <FlatList
        data={singleEvent.samples}
        keyExtractor={(item, index) => `${item.seq}-${index}`}
        ListHeaderComponent={
          <View>
            <Text style={styles.subTitle}>Device event ID: {singleEvent.eventId}</Text>
            <Text style={styles.subTitle}>Tidspunkt: {new Date(singleEvent.savedAt).toLocaleString()}</Text>
            <Text style={styles.subTitle}>Samples: {singleEvent.sampleCount}</Text>
            <Text style={styles.sectionTitle}>gz graf</Text>
            <GzChart samples={singleEvent.samples} meta={singleEvent.meta} />
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
    paddingTop: 0,
    backgroundColor: '#fff',
  },
  logo: {
    fontSize: 28,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: { width: -1, height: 1 },
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#222',
    marginBottom: 8,
    letterSpacing: -0.5,
    flex: 1,
  },
  topBar: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 18,
    gap: 12,
  },
  headerRow: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 18,
    marginBottom: 10,
    gap: 12,
  },
  backButton: {
    width: 24,
    height: 24,
    marginLeft: -6,
    marginRight: 6,
  },
  backButtonPlaceholder: {
    width: 24,
    height: 24,
  },
  pageTitle: {
    fontSize: 20,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#2c2c2c',
    paddingLeft: 18,
    textAlign: 'left',
    marginBottom: 4,
  },
  pageTitleRow: {
    paddingHorizontal: 18,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 4,
  },
  pageTitleInline: {
    flex: 1,
    fontSize: 20,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#2c2c2c',
    marginRight: 8,
  },
  exportToggleButton: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 4,
    backgroundColor: '#303030',
    borderRadius: 999,
    paddingHorizontal: 10,
    paddingVertical: 6,
  },
  exportToggleButtonActive: {
    backgroundColor: '#cc2f2f',
  },
  exportToggleButtonText: {
    fontSize: 12,
    fontFamily: 'RethinkSans_400Regular',
    color: '#fff',
  },
  exportToggleButtonTextActive: {
    color: '#ffffff',
  },
  sessionHeaderBlock: {
    marginBottom: 10,
    paddingHorizontal: 18,
  },
  exportHeaderRow: {
    marginTop: 8,
    paddingBottom: 8,
    borderBottomWidth: 1,
    borderBottomColor: '#d8d8d8',
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  exportHeaderTitle: {
    fontSize: 18,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#2c2c2c',
  },
  exportSelectAllButton: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    marginRight: 11,
  },
  exportSelectAllText: {
    fontSize: 12,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#2c2c2c',
  },
  bottomFadeBlurWrap: {
    position: 'absolute',
    left: 0,
    right: 0,
    bottom: 0,
    height: 230,
    zIndex: 20,
  },
  exportActionBarWrap: {
    position: 'absolute',
    left: 18,
    right: 18,
    bottom: 108,
    zIndex: 30,
  },
  title: {
    fontSize: 20,
    fontFamily: 'Montserrat_600SemiBold',
    marginBottom: 4,
    marginHorizontal: 18,
  },
  subTitle: {
    fontSize: 12,
    fontFamily: 'RethinkSans_400Regular',
    marginBottom: 4,
    opacity: 0.5,
  },
  tabSwitchWrap: {
    position: 'relative',
    marginTop: 8,
    borderRadius: 999,
    backgroundColor: '#f6f3f3',
    flexDirection: 'row',
    padding: TAB_SWITCH_PADDING,
    width: '100%',
  },
  tabSwitchActivePill: {
    position: 'absolute',
    left: TAB_SWITCH_PADDING,
    top: TAB_SWITCH_PADDING,
    bottom: TAB_SWITCH_PADDING,
    borderRadius: 999,
    backgroundColor: '#7aa16f',
  },
  tabSwitchButton: {
    width: '50%',
    minHeight: 27,
    borderRadius: 999,
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 1,
  },
  tabSwitchText: {
    fontSize: 12,
    fontFamily: 'Montserrat_600SemiBold',
    textAlign: 'center',
  },
  summaryCard: {
    borderRadius: 16,
    backgroundColor: '#f6f3f3',
    padding: 12,
    marginBottom: 12,
    marginTop: 6,
    marginHorizontal: 18,
    borderWidth: 1,
    borderColor: '#e1e1e1',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
    boxShadow: 'rgba(100, 100, 111, 0.20) 0px 1px 4px 0px',
  },
  summaryCardTitle: {
    fontSize: 14,
    fontFamily: 'Montserrat_600SemiBold',
    marginBottom: 8,
  },
  graphPlaceholder: {
    height: 180,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#cdcdcd',
    backgroundColor: '#eceaea',
    alignItems: 'center',
    justifyContent: 'center',
  },
  graphPlaceholderText: {
    fontSize: 14,
    opacity: 0.6,
  },
  puttCard: {
    borderRadius: 16,
    backgroundColor: '#f6f3f3',
    borderWidth: 1,
    borderColor: '#e1e1e1',
    marginTop: 6,
    marginHorizontal: 18,
    overflow: 'hidden',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 6,
    elevation: 3,
    boxShadow: 'rgba(100, 100, 111, 0.20) 0px 1px 4px 0px',
  },
  puttCardExpanded: {
    paddingBottom: 10,
  },
  puttRowTop: {
    minHeight: 45,
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 10,
    paddingVertical: 2,
    gap: 10,
  },
  puttNumberBadge: {
    width: 29,
    height: 29,
    borderRadius: 8,
    backgroundColor: '#6f916a',
    alignItems: 'center',
    justifyContent: 'center',
  },
  puttNumberText: {
    color: '#fff',
    fontSize: 16,
    fontFamily: 'Montserrat_600SemiBold',
  },
  puttMetricsRow: {
    flex: 1,
    flexDirection: 'row',
    justifyContent: 'space-between',
    gap: 6,
  },
  puttMetricCol: {
    flex: 1,
    alignItems: 'center',
    marginVertical: 2,
  },
  puttMetricLabel: {
    fontSize: 8,
    opacity: 0.75,
  },
  puttMetricValue: {
    fontSize: 12,
    fontFamily: 'SpecialGothicExpandedOne_400Regular',
    paddingTop: 2,
  },
  puttMetricUnit: {
    fontSize: 6,
    opacity: 0.8,
  },
  chevronIcon: {
    width: 29,
    height: 29,
    textAlign: 'center',
    textAlignVertical: 'center',
    marginTop: 10,
  },
  checkboxBox: {
    width: 27,
    height: 27,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  checkboxBoxSelected: {
    backgroundColor: '#e8f0e3',
    borderWidth: 1,
    borderColor: '#5f8457',
  },
  checkboxBoxUnselected: {
    backgroundColor: 'transparent',
    borderWidth: 1,
    borderColor: '#b5b5b5',
  },
  puttExpandedBlock: {
    marginHorizontal: 10,
    borderTopWidth: 1,
    borderTopColor: '#cdcdcd',
    paddingTop: 10,
  },
  compareRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
    marginBottom: 2,
  },
  compareText: {
    fontSize: 9,
    fontFamily: 'Montserrat_600SemiBold',
  },
  compareToggle: {
    padding: 2,
  },
  compareCheckboxBox: {
    width: 16,
    height: 16,
    borderRadius: 4,
    alignItems: 'center',
    justifyContent: 'center',
  },
  compareCheckboxBoxSelected: {
    backgroundColor: '#e8f0e3',
    borderWidth: 1,
    borderColor: '#5f8457',
  },
  compareCheckboxBoxUnselected: {
    backgroundColor: 'transparent',
    borderWidth: 1,
    borderColor: '#c9c9c9',
  },
  puttChartCard: {
    borderRadius: 16,
    backgroundColor: '#ffffff',
    borderWidth: 1,
    borderColor: '#dedede',
    paddingHorizontal: 12,
    paddingTop: 12,
    paddingBottom: 10,
    marginTop: 6,
    boxShadow: 'rgba(100, 100, 111, 0.18) 0px 1px 6px 0px',
  },
  puttChartHeaderRow: {
    flexDirection: 'row',
    alignItems: 'baseline',
    justifyContent: 'space-between',
    marginBottom: 8,
  },
  puttChartTitle: {
    fontSize: 13,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#1f1f1f',
  },
  puttChartSubtitle: {
    fontSize: 11,
    fontFamily: 'RethinkSans_400Regular',
    color: '#808080',
  },
  puttChartPlot: {
    position: 'relative',
    overflow: 'hidden',
    backgroundColor: '#fbfbfb',
    borderRadius: 12,  },
  puttChartGridLine: {
    position: 'absolute',
    left: 30,
    right: 0,
    height: 1,
    backgroundColor: '#ececec',
  },
  puttChartZeroLine: {
    position: 'absolute',
    left: 30,
    right: 0,
    height: 1,
    backgroundColor: '#bdbdbd',
  },
  puttChartYAxisLabel: {
    position: 'absolute',
    left: 0,
    width: 28,
    textAlign: 'right',
    fontSize: 10,
    color: '#8d8d8d',
  },
  puttChartXAxisLabel: {
    position: 'absolute',
    bottom: 2,
    marginLeft: -12,
    width: 24,
    textAlign: 'center',
    fontSize: 10,
    color: '#8d8d8d',
  },
  puttChartPhaseMarker: {
    position: 'absolute',
    top: 0,
    bottom: 16,
    alignItems: 'center',
    width: 1,
    zIndex: 2,
  },
  puttChartPhaseLabel: {
    fontSize: 10,
    fontFamily: 'RethinkSans_400Regular',
    marginBottom: 4,
    backgroundColor: '#fbfbfb',
    paddingHorizontal: 2,
  },
  puttChartPhaseLine: {
    width: 1,
    flex: 1,
    opacity: 0.75,
  },
  puttChartSegment: {
    position: 'absolute',
    height: 1,
    backgroundColor: '#d53f3f',
    zIndex: 3,
    transformOrigin: 'left center',
  },
  puttChartPoint: {
    position: 'absolute',
    width: 3,
    height: 3,
    borderRadius: 1.5,
    backgroundColor: '#d53f3f',
    zIndex: 4,
  },
  puttChartFootnoteRow: {
    marginTop: 8,
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  puttChartFootnote: {
    fontSize: 10,
    color: '#8a8a8a',
  },
  puttChartEmptyText: {
    fontSize: 12,
    color: '#7f7f7f',
    textAlign: 'center',
    paddingVertical: 16,
  },
  sectionTitle: {
    marginTop: 14,
    marginBottom: 8,
    fontSize: 16,
    fontWeight: '700',
    marginHorizontal: 18,
  },
  chartMeta: {
    fontSize: 12,
    opacity: 0.7,
  },
  chartRow: {
    marginTop: 8,
    height: CHART_HEIGHT,
    position: 'relative',
    borderWidth: 1,
    borderColor: '#ccd0d4',
    borderRadius: 8,
    backgroundColor: '#ffffff',
    overflow: 'hidden',
  },
  zeroLine: {
    position: 'absolute',
    left: 0,
    right: 0,
    height: 1,
    backgroundColor: '#aab2bb',
    opacity: 0.8,
    zIndex: 1,
  },
  markerWrap: {
    position: 'absolute',
    top: 0,
    bottom: 0,
    width: 1,
    zIndex: 2,
    alignItems: 'center',
  },
  markerLine: {
    width: 2,
    flex: 1,
    opacity: 0.8,
  },
  markerLabel: {
    fontSize: 10,
    fontWeight: '700',
    marginBottom: 2,
    backgroundColor: '#ffffff',
    paddingHorizontal: 2,
  },
  lineSegment: {
    position: 'absolute',
    height: 1,
    backgroundColor: '#2f6f3f',
    zIndex: 2,
    transformOrigin: 'left center',
  },
  point: {
    position: 'absolute',
    width: POINT_SIZE,
    height: POINT_SIZE,
    borderRadius: POINT_SIZE / 2,
    backgroundColor: '#2f6f3f',
    zIndex: 3,
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
  listContentWithExportBar: {
    paddingBottom: 80,
  },
  exportActionButton: {
    backgroundColor: '#303030',
    borderRadius: 999,
    zIndex: 30,
    paddingVertical: 14,
    alignItems: 'center',
    justifyContent: 'center',
  },
  exportActionButtonDisabled: {
    backgroundColor: '#a0a0a0',
  },
  exportActionButtonText: {
    fontSize: 16,
    color: '#fff',
    fontFamily: 'Montserrat_600SemiBold',
  },
  emptyText: {
    marginTop: 10,
    marginHorizontal: 18,
    opacity: 0.8,
  },
  errorText: {
    marginTop: 10,
    color: '#b00020',
  },
})

export default ActivityDetails
