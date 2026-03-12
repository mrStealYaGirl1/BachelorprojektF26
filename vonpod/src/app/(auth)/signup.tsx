import React, { useMemo, useState } from 'react'
import { Alert, ImageBackground, Pressable, StyleSheet, Text, TextInput, View } from 'react-native'
import { router } from 'expo-router'
import { supabase } from '../../lib/supabase'

export default function SignUpScreen() {
	const [step, setStep] = useState<1 | 2>(1)
	const [email, setEmail] = useState('')
	const [username, setUsername] = useState('')
	const [password, setPassword] = useState('')
	const [confirmPassword, setConfirmPassword] = useState('')
    const [loading, setLoading] = useState(false)

	const stepLabel = useMemo(() => `Sign Up ${step}/2`, [step])

	const goToStepTwo = () => {
		if (!email.trim()) {
			Alert.alert('Fejl', 'Indtast din email')
			return
		}

		if (!email.includes('@')) {
			Alert.alert('Fejl', 'Indtast en gyldig email')
			return
		}

		setStep(2)
	}

	const createAccount = async () => {
		if (!username.trim() || !password || !confirmPassword) {
			Alert.alert('Fejl', 'Udfyld username og begge password felter')
			return
		}

		if (password.length < 6) {
			Alert.alert('Fejl', 'Password skal mindst være 6 tegn')
			return
		}

		if (password !== confirmPassword) {
			Alert.alert('Fejl', 'Passwords matcher ikke')
			return
		}

		setLoading(true)

		const { data, error } = await supabase.auth.signUp({
			email: email.trim(),
			password,
			options: {
				data: {
					username: username.trim(),
				},
			},
		})

		setLoading(false)

		if (error) {
			Alert.alert('Signup fejl', error.message)
			return
		}

		if (data.session) {
			Alert.alert('Succes', 'Din konto er oprettet')
			router.replace('/(tabs)')
			return
		}

		Alert.alert('Konto oprettet', 'Log ind med din nye bruger for at fortsætte.')
		router.replace('/(auth)/login')
	}

	return (
		<ImageBackground
			source={require('../../../assets/images/login_background.png')}
			style={styles.container}
			resizeMode="cover"
		>
			<View style={styles.overlay} />
			<View style={styles.sheet}>
				<View style={styles.headerRow}>
					<Pressable onPress={() => (step === 1 ? router.back() : setStep(1))}>
						<Text style={styles.backText}>←</Text>
					</Pressable>
					<Text style={styles.title}>Sign Up</Text>
					<Text style={styles.stepText}>{stepLabel}</Text>
				</View>

				{step === 1 ? (
					<>
						<TextInput
							placeholder="Enter your email"
							autoCapitalize="none"
							keyboardType="email-address"
							value={email}
							onChangeText={setEmail}
							style={styles.input}
							placeholderTextColor="#7f7f7f"
						/>

						<Pressable style={styles.primaryButton} onPress={goToStepTwo}>
							<Text style={styles.primaryButtonText}>Next</Text>
						</Pressable>

						<View style={styles.separatorRow}>
							<View style={styles.separatorLine} />
							<Text style={styles.separatorText}>or</Text>
							<View style={styles.separatorLine} />
						</View>

						<Pressable style={styles.socialButton}>
							<Text style={styles.socialButtonText}>Sign up with Apple</Text>
						</Pressable>
						<Pressable style={styles.socialButton}>
							<Text style={styles.socialButtonText}>Sign up with Google</Text>
						</Pressable>
						<Pressable style={styles.socialButton}>
							<Text style={styles.socialButtonText}>Sign up with Facebook</Text>
						</Pressable>
					</>
				) : (
					<>
						<TextInput
							placeholder="Enter password"
							autoCapitalize="none"
							secureTextEntry
							value={password}
							onChangeText={setPassword}
							style={styles.input}
							placeholderTextColor="#7f7f7f"
						/>

						<TextInput
							placeholder="Confirm password"
							autoCapitalize="none"
							secureTextEntry
							value={confirmPassword}
							onChangeText={setConfirmPassword}
							style={styles.input}
							placeholderTextColor="#7f7f7f"
						/>

						<TextInput
							placeholder="Enter a username"
							autoCapitalize="none"
							value={username}
							onChangeText={setUsername}
							style={styles.input}
							placeholderTextColor="#7f7f7f"
						/>

						<Pressable
              style={[styles.primaryButton, loading && styles.buttonDisabled]}
              onPress={createAccount}
              disabled={loading}
            >
							<Text style={styles.primaryButtonText}>{loading ? 'Opretter...' : 'Next'}</Text>
						</Pressable>
					</>
				)}
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
		paddingTop: 24,
	},
	headerRow: {
		flexDirection: 'row',
		alignItems: 'center',
		justifyContent: 'space-between',
		marginBottom: 18,
	},
	backText: {
		fontSize: 24,
		color: '#505050',
		width: 28,
	},
	title: {
		fontSize: 32,
		fontWeight: '700',
		color: '#101010',
	},
	stepText: {
		fontSize: 14,
		color: '#666666',
		width: 70,
		textAlign: 'right',
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
  buttonDisabled: {
    opacity: 0.7,
  },
	separatorRow: {
		marginVertical: 16,
		flexDirection: 'row',
		alignItems: 'center',
		gap: 10,
	},
	separatorLine: {
		flex: 1,
		height: 1,
		backgroundColor: '#d8d8d8',
	},
	separatorText: {
		color: '#767676',
	},
	socialButton: {
		minHeight: 50,
		borderRadius: 999,
		backgroundColor: '#ffffff',
		borderWidth: 1,
		borderColor: '#dcdcdc',
		justifyContent: 'center',
		alignItems: 'center',
		marginBottom: 10,
	},
	socialButtonText: {
		color: '#111111',
		fontSize: 18,
		fontWeight: '500',
	},
})
