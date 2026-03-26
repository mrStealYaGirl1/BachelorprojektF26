import React from 'react'
import { ImageBackground, Pressable, StyleSheet, Text, View } from 'react-native'
import { router } from 'expo-router'

export default function AuthLandingScreen() {
  return (
    <ImageBackground
      source={require('../../../assets/images/login_background.png')}
      style={styles.container}
      resizeMode="cover"
    >
      <View style={styles.overlay} />

      <View style={styles.content}>
        <Text style={styles.brand}>vonpod</Text>

        <View style={styles.buttonGroup}>
          <Pressable style={styles.button} onPress={() => router.push('/login')}>
            <Text style={styles.buttonText}>Login</Text>
          </Pressable>

          <Pressable style={styles.button} onPress={() => router.push('/signup')}>
            <Text style={styles.buttonText}>Create an Account</Text>
          </Pressable>
        </View>
      </View>
    </ImageBackground>
  )
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#6f8969',
  },
  overlay: {
    ...StyleSheet.absoluteFillObject,
    backgroundColor: 'rgba(22, 38, 19, 0.30)',
  },
  content: {
    flex: 1,
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 24,
    paddingTop: 140,
    paddingBottom: 56,
  },
  brand: {
    color: '#ffffff',
    fontSize: 52,
    fontWeight: '800',
    letterSpacing: -1,
  },
  buttonGroup: {
    width: '100%',
    gap: 14,
  },
  button: {
    backgroundColor: '#ffffff',
    borderRadius: 999,
    minHeight: 54,
    justifyContent: 'center',
    alignItems: 'center',
  },
  buttonText: {
    color: '#111111',
    fontSize: 24,
    fontWeight: '500',
  },
})