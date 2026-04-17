import React, { useCallback, useMemo, useState } from 'react'
import { Pressable, SafeAreaView, ScrollView, StyleSheet, Text, View } from 'react-native'
import Ionicons from '@expo/vector-icons/Ionicons'
import { useFocusEffect } from '@react-navigation/native'
import { useRouter } from 'expo-router'
import { useAuth } from '../../providers/AuthProvider'
import { loadStoredEvents } from '../../lib/imuStorage'
import { loadStoredTrainingSessions } from '../../lib/trainingStorage'
import { fetchOwnProfile, type ProfileRow } from '../../lib/profile'

type ProfileStats = {
  totalPutts: number
  totalSessions: number
  totalTrainingSeconds: number
}

function formatDuration(totalSeconds: number): string {
  if (!Number.isFinite(totalSeconds) || totalSeconds <= 0) {
    return '0h 0m'
  }

  const hours = Math.floor(totalSeconds / 3600)
  const minutes = Math.floor((totalSeconds % 3600) / 60)
  return `${hours}h ${minutes}m`
}

function formatAccountAge(createdAt: string | undefined): string {
  if (!createdAt) return '-'

  const createdDate = new Date(createdAt)
  if (Number.isNaN(createdDate.getTime())) return '-'

  const now = new Date()
  const months =
    (now.getFullYear() - createdDate.getFullYear()) * 12 +
    (now.getMonth() - createdDate.getMonth())

  if (months < 1) {
    const days = Math.max(
      0,
      Math.floor((now.getTime() - createdDate.getTime()) / (1000 * 60 * 60 * 24))
    )
    return `${days}d`
  }

  if (months < 12) return `${months}m`
  return `${(months / 12).toFixed(1)}y`
}

export default function Profile() {
  const { session } = useAuth()
  const router = useRouter()
  const [stats, setStats] = useState<ProfileStats>({
    totalPutts: 0,
    totalSessions: 0,
    totalTrainingSeconds: 0,
  })
  const [profile, setProfile] = useState<ProfileRow | null>(null)
  const [statsError, setStatsError] = useState<string | null>(null)
  const [profileError, setProfileError] = useState<string | null>(null)

  useFocusEffect(
    useCallback(() => {
      let active = true

      const loadStats = async () => {
        try {
          const [events, sessions] = await Promise.all([
            loadStoredEvents(),
            loadStoredTrainingSessions(),
          ])
          if (!active) return

          const activeUserId = session?.user?.id ?? null
          const userSessions = sessions.filter((trainingSession) =>
            activeUserId ? trainingSession.userId === activeUserId : true
          )

          const totalTrainingSeconds = userSessions.reduce((sum, trainingSession) => {
            const duration = trainingSession.durationSeconds
            if (!Number.isFinite(duration) || duration <= 0) {
              return sum
            }

            return sum + duration
          }, 0)

          setStats({
            totalPutts: events.length,
            totalSessions: userSessions.length,
            totalTrainingSeconds,
          })
          setStatsError(null)
        } catch (error) {
          if (!active) return
          const message = error instanceof Error ? error.message : 'Kunne ikke indlaese statistik'
          setStatsError(message)
        }
      }

      void loadStats()

      return () => {
        active = false
      }
    }, [session?.user?.id])
  )

  useFocusEffect(
    useCallback(() => {
      let active = true

      const loadProfile = async () => {
        if (!session?.user) return

        try {
          const data = await fetchOwnProfile(session.user.id)
          if (!active) return
          setProfile(data)
          setProfileError(null)
        } catch (error) {
          if (!active) return
          const message =
            error instanceof Error ? error.message : 'Kunne ikke indlaese profiloplysninger'
          setProfileError(message)
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

  const fullNameFromMetadata = firstName && lastName ? `${firstName} ${lastName}` : null

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

  const displayName = fullNameFromProfile ?? fullNameFromMetadata ?? fallbackUsername
  const displayHandle = `@${fallbackUsername}`

  const accountAge = useMemo(() => formatAccountAge(user?.created_at), [user?.created_at])

  return (
    <SafeAreaView style={styles.safeArea}>
      <ScrollView contentContainerStyle={styles.contentContainer}>
        <View style={styles.topCard}>
          <View style={styles.headerArea}>
            <Text style={styles.brandText}>vonpod</Text>
            <Pressable style={styles.headerIconButton} onPress={() => router.push('/settings')}>
              <Ionicons name='settings-outline' size={26} color='#1f2328' />
            </Pressable>
          </View>

          <View style={styles.avatar}>
            <Ionicons name='person' size={88} color='#a6a6a6' />
          </View>

          <Text style={styles.nameText}>{displayName}</Text>
          <Text style={styles.handleText}>{displayHandle}</Text>

          <View style={styles.statsCard}>
            <View style={styles.statsRow}>
              <View style={styles.statsItem}>
                <Text style={styles.statsLabel}>Sessions</Text>
                <Text style={styles.statsValue}>{stats.totalSessions}</Text>
              </View>

              <View style={styles.statsItem}>
                <Text style={styles.statsLabel}>Total Putts</Text>
                <Text style={styles.statsValue}>{stats.totalPutts}</Text>
              </View>

              <View style={styles.statsItem}>
                <Text style={styles.statsLabel}>Training Time</Text>
                <Text style={styles.statsValue}>{formatDuration(stats.totalTrainingSeconds)}</Text>
              </View>
            </View>

            <View style={styles.statsRow}>
              <View style={styles.statsItem}>
                <Text style={styles.statsLabel}>Plan</Text>
                <Text style={styles.statsValue}>Free</Text>
              </View>

              <View style={styles.statsItem}>
                <Text style={styles.statsLabel}>Account age</Text>
                <Text style={styles.statsValue}>{accountAge}</Text>
              </View>

              <View style={styles.statsItem} />
            </View>
          </View>

          {!!statsError && <Text style={styles.errorText}>{statsError}</Text>}
          {!!profileError && <Text style={styles.errorText}>{profileError}</Text>}

          <Pressable style={styles.actionCard} onPress={() => router.push('/settings')}>
            <Text style={styles.actionText}>Profilindstillinger</Text>
            <Ionicons name='chevron-forward' size={18} color='#5e5e5e' />
          </Pressable>

          <Pressable style={styles.actionCard}>
            <Text style={styles.actionText}>Kontaktoplysninger</Text>
            <Ionicons name='chevron-forward' size={18} color='#5e5e5e' />
          </Pressable>

          <Pressable style={styles.actionCard}>
            <Text style={styles.actionText}>Hjaelp</Text>
            <Ionicons name='chevron-forward' size={18} color='#5e5e5e' />
          </Pressable>
        </View>
      </ScrollView>
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
  },
  contentContainer: {
    paddingBottom: 120,
    backgroundColor: '#191919',
  },
  topCard: {
    backgroundColor: '#f3f3f3',
    borderBottomLeftRadius: 42,
    borderBottomRightRadius: 42,
    paddingHorizontal: 24,
    paddingBottom: 26,
    minHeight: '100%',
  },
  headerArea: {
    position: 'relative',
    alignItems: 'center',
    justifyContent: 'center',
    minHeight: 40,
  },
  brandText: {
    fontSize: 28,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: { width: -1, height: 1 },
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#222',
    marginBottom: 4,
  },
  headerIconButton: {
    position: 'absolute',
    right: 0,
    top: 0,
    width: 40,
    height: 40,
    borderRadius: 20,
    justifyContent: 'center',
    alignItems: 'center',
  },
  avatar: {
    marginTop: 18,
    alignSelf: 'center',
    width: 160,
    height: 160,
    borderRadius: 80,
    backgroundColor: '#d8d8d8',
    justifyContent: 'center',
    alignItems: 'center',
  },
  nameText: {
    marginTop: 20,
    textAlign: 'center',
    fontSize: 24,
    fontWeight: '800',
    color: '#0f1113',
  },
  handleText: {
    marginTop: 6,
    textAlign: 'center',
    fontSize: 16,
    color: '#6f6f6f',
    fontWeight: '500',
  },
  statsCard: {
    marginTop: 24,
    borderRadius: 26,
    backgroundColor: '#e9e5e4',
    paddingVertical: 20,
    paddingHorizontal: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 10,
    elevation: 3,
    boxShadow: 'rgba(100, 100, 111, 0.35) 0px 1px 8px 0px',
  },
  statsRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 14,
  },
  statsItem: {
    flex: 1,
    alignItems: 'center',
    minHeight: 66,
  },
  statsLabel: {
    fontSize: 12,
    color: '#232323',
    marginBottom: 6,
  },
  statsValue: {
    fontSize: 20,
    fontWeight: '800',
    color: '#08090a',
  },
  actionCard: {
    marginTop: 16,
    borderRadius: 24,
    backgroundColor: '#e9e5e4',
    minHeight: 60,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 20,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 10,
    elevation: 3,
    boxShadow: 'rgba(100, 100, 111, 0.35) 0px 1px 8px 0px',
  },
  actionText: {
    fontSize: 20,
    fontWeight: '600',
    color: '#151618',
  },
  errorText: {
    marginTop: 12,
    fontSize: 13,
    color: '#b00020',
    textAlign: 'center',
  },
})