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
    paddingTop: 120,
    paddingBottom: 56,
  },
  brand: {
    fontSize: 56,
    letterSpacing: -1,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: { width: -1, height: 1 },
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#fff',
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
    marginHorizontal: 10,
  },
  buttonText: {
    color: '#111111',
    fontSize: 18,
    fontFamily: 'RethinkSans_600SemiBold',
    fontWeight: '500',
  },
})