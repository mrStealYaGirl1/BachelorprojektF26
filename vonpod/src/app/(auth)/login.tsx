import React, { useState } from 'react'
import { Alert, ImageBackground, Pressable, StyleSheet, Text, TextInput, View } from 'react-native'
import { supabase } from '../../lib/supabase'
import { router } from 'expo-router'

export default function AuthScreen() {
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const [loading, setLoading] = useState(false)

  const signIn = async () => {
    if (!email || !password) {
      Alert.alert('Fejl', 'Udfyld email og password')
      return
    }

    setLoading(true)

    const { error } = await supabase.auth.signInWithPassword({
      email,
      password,
    })

    setLoading(false)

    if (error) {
      Alert.alert('Login fejl', error.message)
      return
    }

    Alert.alert('Succes', 'Du er logget ind')
    router.replace('/home')
  }

  return (
    <ImageBackground
      source={require('../../../assets/images/login_background.png')}
      style={styles.container}
      resizeMode="cover"
    >
      <View style={styles.overlay} />

      <View style={styles.sheet}>
        <Text style={styles.title}>Login</Text>

        <TextInput
          placeholder="Enter email"
          autoCapitalize="none"
          keyboardType="email-address"
          value={email}
          onChangeText={setEmail}
          style={styles.input}
          placeholderTextColor="#7f7f7f"
        />

        <TextInput
          placeholder="Enter password"
          autoCapitalize="none"
          secureTextEntry
          value={password}
          onChangeText={setPassword}
          style={styles.input}
          placeholderTextColor="#7f7f7f"
        />

        <Pressable style={[styles.primaryButton, loading && styles.buttonDisabled]} onPress={signIn} disabled={loading}>
          <Text style={styles.primaryButtonText}>{loading ? 'Logger ind...' : 'Login'}</Text>
        </Pressable>

        <Pressable style={styles.secondaryButton} onPress={() => router.push('/signup')}>
          <Text style={styles.secondaryButtonText}>Create an account</Text>
        </Pressable>
      </View>
    </ImageBackground>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  overlay: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: 'rgba(22, 38, 19, 0.30)',
  },
  sheet: {
    marginTop: '50%',
    flex: 1,
    backgroundColor: '#f7f7f7',
    borderTopLeftRadius: 36,
    borderTopRightRadius: 36,
    paddingHorizontal: 24,
    paddingTop: 28,
  },
  title: {
    fontSize: 32,
    fontWeight: '700',
    color: '#101010',
    marginBottom: 18,
  },
  input: {
    borderWidth: 1,
    borderColor: '#d1d1d1',
    borderRadius: 24,
    paddingHorizontal: 16,
    minHeight: 48,
    backgroundColor: '#f1f1f1',
    marginBottom: 12,
  },
  primaryButton: {
    marginTop: 8,
    minHeight: 52,
    borderRadius: 999,
    backgroundColor: '#5e7f56',
    justifyContent: 'center',
    alignItems: 'center',
  },
  primaryButtonText: {
    color: '#ffffff',
    fontSize: 20,
    fontWeight: '600',
  },
  secondaryButton: {
    marginTop: 12,
    minHeight: 52,
    borderRadius: 999,
    backgroundColor: '#ffffff',
    borderWidth: 1,
    borderColor: '#e1e1e1',
    justifyContent: 'center',
    alignItems: 'center',
  },
  secondaryButtonText: {
    color: '#111111',
    fontSize: 18,
    fontWeight: '500',
  },
  buttonDisabled: {
    opacity: 0.7,
  },
})