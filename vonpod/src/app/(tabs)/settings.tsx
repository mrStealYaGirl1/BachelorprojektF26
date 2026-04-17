import { View, Text, Alert, Pressable, StyleSheet, TextInput, SafeAreaView } from 'react-native'
import React, { useCallback, useState } from 'react'
import { useFocusEffect } from '@react-navigation/native'
import { useAuth } from '../../providers/AuthProvider'
import { fetchOwnProfile, upsertOwnProfileNames } from '../../lib/profile'

const Settings = () => {
  const { signOut, session } = useAuth()
  const [loading, setLoading] = useState(false)
  const [saving, setSaving] = useState(false)
  const [firstName, setFirstName] = useState('')
  const [lastName, setLastName] = useState('')
  const [username, setUsername] = useState<string | undefined>(undefined)

  useFocusEffect(
    useCallback(() => {
      let active = true

      const loadProfile = async () => {
        if (!session?.user) return

        try {
          const profile = await fetchOwnProfile(session.user.id)
          if (!active) return

          setFirstName(profile?.first_name ?? '')
          setLastName(profile?.last_name ?? '')
          setUsername(profile?.username)
        } catch (error) {
          const message =
            error instanceof Error ? error.message : 'Kunne ikke indlaese profiloplysninger'
          Alert.alert('Fejl', message)
        }
      }

      void loadProfile()

      return () => {
        active = false
      }
    }, [session?.user])
  )

  const handleSaveProfile = async () => {
    if (!session?.user) {
      Alert.alert('Fejl', 'Ingen aktiv bruger fundet')
      return
    }

    try {
      setSaving(true)
      await upsertOwnProfileNames({
        user: session.user,
        username,
        firstName,
        lastName,
      })
      Alert.alert('Gemt', 'Profiloplysninger er opdateret')
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke gemme profiloplysninger'
      Alert.alert('Fejl', message)
    } finally {
      setSaving(false)
    }
  }

  const handleSignOut = async () => {
    try {
      setLoading(true)
      await signOut()
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke logge ud'
      Alert.alert('Fejl', message)
    } finally {
      setLoading(false)
    }
  }

  return (
    <SafeAreaView style={styles.screen}>
      <View style={styles.container}>
        <Text style={styles.title}>Settings</Text>

        <View style={styles.inputGroup}>
          <Text style={styles.label}>Fornavn</Text>
          <TextInput
            style={styles.input}
            value={firstName}
            onChangeText={setFirstName}
            autoCapitalize='words'
            placeholder='Indtast fornavn'
            placeholderTextColor='#8a8a8a'
          />
        </View>

        <View style={styles.inputGroup}>
          <Text style={styles.label}>Efternavn</Text>
          <TextInput
            style={styles.input}
            value={lastName}
            onChangeText={setLastName}
            autoCapitalize='words'
            placeholder='Indtast efternavn'
            placeholderTextColor='#8a8a8a'
          />
        </View>

        <Pressable
          style={[styles.button, saving && styles.buttonDisabled]}
          onPress={handleSaveProfile}
          disabled={saving}
        >
          <Text style={styles.buttonText}>{saving ? 'Gemmer...' : 'Gem profil'}</Text>
        </Pressable>

        <Pressable
          style={[styles.button, styles.signOutButton, loading && styles.buttonDisabled]}
          onPress={handleSignOut}
          disabled={loading}
        >
          <Text style={styles.buttonText}>{loading ? 'Logger ud...' : 'Log ud'}</Text>
        </Pressable>
      </View>
    </SafeAreaView>
  )
}

const styles = StyleSheet.create({
  screen: {
    flex: 1,
    backgroundColor: '#f4f3f1',
  },
  container: {
    flex: 1,
    paddingHorizontal: 24,
    paddingTop: 24,
  },
  title: {
    fontSize: 30,
    fontWeight: '700',
    color: '#111',
    marginBottom: 24,
  },
  inputGroup: {
    marginBottom: 14,
  },
  label: {
    fontSize: 15,
    color: '#2f2f2f',
    marginBottom: 8,
    fontWeight: '500',
  },
  input: {
    borderWidth: 1,
    borderColor: '#d1d1d1',
    borderRadius: 16,
    paddingHorizontal: 14,
    minHeight: 48,
    backgroundColor: '#ffffff',
    fontSize: 16,
    color: '#131313',
  },

  button: {
    marginTop: 8,
    minHeight: 52,
    borderRadius: 16,
    backgroundColor: '#5e7f56',
    alignItems: 'center',
    justifyContent: 'center',
  },
  signOutButton: {
    marginTop: 12,
    backgroundColor: '#324744',
  },

  buttonDisabled: {
    opacity: 0.7,
  },

  buttonText: {
    color: '#FFFFFF',
    fontSize: 18,
    fontWeight: '600',
    textAlign: 'center',
  },
})

export default Settings