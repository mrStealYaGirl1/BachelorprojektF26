import React, { useCallback, useEffect, useState } from 'react'
import {
  Modal,
  Pressable,
  SafeAreaView,
  ScrollView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native'
import { useRouter } from 'expo-router'
import { useFocusEffect } from '@react-navigation/native'
import Ionicons from '@expo/vector-icons/Ionicons'
import {
  loadStoredTrainingSessions,
  type StoredTrainingSession,
  updateStoredTrainingSession,
} from '../../lib/trainingStorage'

const HARDCODED_LOCATION = 'Aarhus Golf Club'

function formatDuration(totalSeconds: number): string {
  const seconds = Math.max(0, Math.floor(totalSeconds))
  const minutes = Math.floor(seconds / 60)
  const remainingSeconds = seconds % 60
  return `${minutes}m ${remainingSeconds.toString().padStart(2, '0')}s`
}

function toDateBadgeParts(isoDate: string): { month: string; day: string } {
  const date = new Date(isoDate)
  const month = date
    .toLocaleString('en-US', { month: 'short' })
    .toUpperCase()
  const day = date.getDate().toString().padStart(2, '0')
  return { month, day }
}

const Activities = () => {
  const router = useRouter()
  const [sessions, setSessions] = useState<StoredTrainingSession[]>([])
  const [errorText, setErrorText] = useState<string | null>(null)
  const [isLoading, setIsLoading] = useState(false)
  const [renameValue, setRenameValue] = useState('')
  const [renamingSessionId, setRenamingSessionId] = useState<string | null>(null)
  const [isRenameModalVisible, setIsRenameModalVisible] = useState(false)
  const [isSavingRename, setIsSavingRename] = useState(false)

  const getDefaultActivityName = useCallback((startedAt: string) => {
    const date = new Date(startedAt)
    const dateLabel = date.toLocaleDateString('en-GB', {
      day: '2-digit',
      month: 'short',
      year: 'numeric',
    })
    return `Training session ${dateLabel}`
  }, [])

  const loadSessions = useCallback(async () => {
    setIsLoading(true)
    setErrorText(null)

    try {
      const parsed = await loadStoredTrainingSessions()
      const sorted = [...parsed].sort(
        (a, b) => new Date(b.startedAt).getTime() - new Date(a.startedAt).getTime()
      )
      setSessions(sorted)
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke indlaese sessions'
      setErrorText(message)
    } finally {
      setIsLoading(false)
    }
  }, [])

  useEffect(() => {
    void loadSessions()
  }, [loadSessions])

  useFocusEffect(
    useCallback(() => {
      void loadSessions()
    }, [loadSessions])
  )

  const openRenameModal = (trainingSession: StoredTrainingSession) => {
    const currentName = trainingSession.activityName?.trim() || getDefaultActivityName(trainingSession.startedAt)
    setRenameValue(currentName)
    setRenamingSessionId(trainingSession.id)
    setIsRenameModalVisible(true)
  }

  const closeRenameModal = () => {
    if (isSavingRename) return
    setIsRenameModalVisible(false)
    setRenamingSessionId(null)
    setRenameValue('')
  }

  const saveRenamedActivity = async () => {
    if (!renamingSessionId) return

    setIsSavingRename(true)
    setErrorText(null)

    try {
      const trimmedName = renameValue.trim()
      const persistedName = trimmedName.length > 0 ? trimmedName : undefined

      const nextSessions = await updateStoredTrainingSession(renamingSessionId, {
        activityName: persistedName,
      })

      const sorted = [...nextSessions].sort(
        (a, b) => new Date(b.startedAt).getTime() - new Date(a.startedAt).getTime()
      )
      setSessions(sorted)
      closeRenameModal()
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke gemme aktivitetsnavn'
      setErrorText(message)
    } finally {
      setIsSavingRename(false)
    }
  }

  return (
    <SafeAreaView style={styles.container}>
      <Text style={styles.logo}>vonpod</Text>
      <Text style={styles.title}>Activities</Text>
      <Text style={styles.subtitle}>{isLoading ? 'Indlaeser...' : `${sessions.length} sessions`}</Text>

      {!!errorText && <Text style={styles.errorText}>{errorText}</Text>}

      <ScrollView contentContainerStyle={styles.listContent}>
        {sessions.length === 0 ? (
          <Text style={styles.emptyText}>Ingen sessions endnu. Start en traening for at oprette den foerste session.</Text>
        ) : (
          sessions.map((trainingSession) => {
            const badge = toDateBadgeParts(trainingSession.startedAt)
            const activityName =
              trainingSession.activityName?.trim() || getDefaultActivityName(trainingSession.startedAt)

            return (
              <Pressable
                key={trainingSession.id}
                style={styles.card}
                onPress={() =>
                  router.push({
                    pathname: '/activity-details',
                    params: { sessionId: trainingSession.id },
                  })
                }
              >
                <View style={styles.dateBadge}>
                  <Text style={styles.dateMonth}>{badge.month}</Text>
                  <Text style={styles.dateDay}>{badge.day}</Text>
                </View>

                <View style={styles.contentGrid}>
                  <View style={styles.leftColumn}>
                    <View style={styles.cardTitleRow}>
                      <Text numberOfLines={1} style={styles.activityNameText}>{activityName}</Text>
                      <Pressable
                        onPress={(event) => {
                          event.stopPropagation()
                          openRenameModal(trainingSession)
                        }}
                        hitSlop={8}
                        style={styles.renameButton}
                      >
                        <Ionicons name='pencil-outline' size={12} color='#8a8a8a' />
                      </Pressable>
                    </View>

                    <View style={styles.locationRow}>
                      <Ionicons name='location-outline' size={14} color='#7a7a7a' />
                      <Text numberOfLines={1} style={styles.locationText}>{HARDCODED_LOCATION}</Text>
                    </View>
                  </View>

                  <View style={styles.rightColumn}>
                    <View style={styles.metricRow}>
                      <Ionicons name='time-outline' size={16} color='#1f1f1f' />
                      <Text style={styles.metricValueText}>{formatDuration(trainingSession.durationSeconds)}</Text>
                    </View>

                    <View style={styles.metricRow}>
                      <Ionicons name='golf-outline' size={16} color='#1f1f1f' />
                      <Text style={styles.metricValueText}>{trainingSession.puttsCount} putts</Text>
                    </View>
                  </View>
                </View>

                <Ionicons name='chevron-forward' size={24} color='#b8b8b8' />
              </Pressable>
            )
          })
        )}
      </ScrollView>

      <Modal
        animationType='fade'
        transparent
        visible={isRenameModalVisible}
        onRequestClose={closeRenameModal}
      >
        <View style={styles.renameBackdrop}>
          <View style={styles.renameCard}>
            <Text style={styles.renameTitle}>Rename activity</Text>
            <TextInput
              style={styles.renameInput}
              value={renameValue}
              onChangeText={setRenameValue}
              placeholder='Activity name'
              placeholderTextColor='#8d8d8d'
              autoFocus
              maxLength={80}
            />

            <View style={styles.renameActionsRow}>
              <Pressable style={styles.renameActionSecondary} onPress={closeRenameModal}>
                <Text style={styles.renameActionSecondaryText}>Cancel</Text>
              </Pressable>

              <Pressable
                style={[styles.renameActionPrimary, isSavingRename && styles.renameActionDisabled]}
                onPress={() => void saveRenamedActivity()}
                disabled={isSavingRename}
              >
                <Text style={styles.renameActionPrimaryText}>Save</Text>
              </Pressable>
            </View>
          </View>
        </View>
      </Modal>
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#ffffff',
    paddingHorizontal: 18,
    paddingTop: 56,
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
  },
  title: {
    fontSize: 24,
    fontFamily: 'RethinkSans_500Medium',

    marginBottom: 2,
    marginLeft: 24,
    color: '#0f0f0f',
  },
  subtitle: {
    fontSize: 15,
    opacity: 0.8,
    marginLeft: 24,
    color: '#565656',
  },
  listContent: {
    paddingBottom: 140,
    paddingTop: 4,
  },
  emptyText: {
    marginTop: 12,
    fontSize: 14,
    opacity: 0.8,
    color: '#4a4a4a',
  },
  card: {
    borderRadius: 20,
    minHeight: 68,
    marginTop: 14,
    marginHorizontal: 18,
    backgroundColor: '#f6f3f3',
    paddingHorizontal: 14,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 14,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 10,
    elevation: 3,
    boxShadow: 'rgba(100, 100, 111, 0.35) 0px 1px 8px 0px',
  },
  contentGrid: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    minWidth: 0,
    paddingVertical: 8,
    gap: 10,
  },
  leftColumn: {
    flex: 1,
    minWidth: 0,
    justifyContent: 'center',
  },
  rightColumn: {
    minWidth: 70,
    gap: 8,
    alignItems: 'flex-start',
  },
  cardTitleRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
    marginBottom: 6,
  },
  activityNameText: {
    flex: 1,
    fontSize: 16,
    paddingLeft: 2,
    color: '#0f0f0f',
    fontFamily: 'Montserrat_600SemiBold',
  },
  renameButton: {
    width: 18,
    height: 18,
    alignItems: 'center',
    justifyContent: 'center',
  },
  locationRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 4,
  },
  locationText: {
    color: '#767676',
    fontSize: 12,
    fontFamily: 'RethinkSans_400Regular',
  },
  metricRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  dateBadge: {
    width: 38,
    height: 38,
    borderRadius: 10,
    paddingTop: 4,
    backgroundColor: '#6f916a',
    alignItems: 'center',
    justifyContent: 'center',
  },
  dateMonth: {
    color: '#ffffff',
    fontSize: 11,
    lineHeight: 11,
    letterSpacing: 0.5,
    paddingTop: 6,
    fontFamily: 'RethinkSans_400Regular',
  },
  dateDay: {
    color: '#ffffff',
    fontSize: 22,
    lineHeight: 23,
    paddingBottom: 3,
    fontFamily: 'RethinkSans_800ExtraBold',
  },
  metricValueText: {
    fontSize: 12,
    color: '#202020',
    fontFamily: 'RethinkSans_400Regular',
  },
  errorText: {
    color: '#b00020',
    marginBottom: 8,
  },
  renameBackdrop: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.3)',
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 24,
  },
  renameCard: {
    width: '100%',
    maxWidth: 360,
    borderRadius: 20,
    backgroundColor: '#f7f6f4',
    padding: 18,
  },
  renameTitle: {
    fontSize: 18,
    color: '#131313',
    fontFamily: 'Montserrat_600SemiBold',
    marginBottom: 10,
  },
  renameInput: {
    borderWidth: 1,
    borderColor: '#d0cdca',
    borderRadius: 12,
    paddingHorizontal: 12,
    height: 44,
    fontSize: 15,
    color: '#1a1a1a',
    backgroundColor: '#ffffff',
  },
  renameActionsRow: {
    marginTop: 14,
    flexDirection: 'row',
    justifyContent: 'flex-end',
    gap: 10,
  },
  renameActionSecondary: {
    minHeight: 38,
    paddingHorizontal: 14,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#e4e2de',
  },
  renameActionSecondaryText: {
    color: '#4f4f4f',
    fontFamily: 'Montserrat_500Medium',
    fontSize: 14,
  },
  renameActionPrimary: {
    minHeight: 38,
    paddingHorizontal: 16,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#3048d2',
  },
  renameActionDisabled: {
    opacity: 0.6,
  },
  renameActionPrimaryText: {
    color: '#ffffff',
    fontFamily: 'Montserrat_600SemiBold',
    fontSize: 14,
  },
})

export default Activities