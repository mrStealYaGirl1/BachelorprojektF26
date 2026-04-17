import React, { useState } from 'react'
import { Alert, ImageBackground, Pressable, StyleSheet, Text, TextInput, View } from 'react-native'
import { router } from 'expo-router'
import { supabase } from '../../lib/supabase'
import Ionicons from '@expo/vector-icons/Ionicons'

export default function SignUpScreen() {
	const [step, setStep] = useState<1 | 2>(1)
	const [email, setEmail] = useState('')
	const [username, setUsername] = useState('')
	const [password, setPassword] = useState('')
	const [confirmPassword, setConfirmPassword] = useState('')
  const [showPassword, setShowPassword] = useState(false)
  const [showConfirmPassword, setShowConfirmPassword] = useState(false)
    const [loading, setLoading] = useState(false)

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
			router.replace('/home')
			return
		}

		Alert.alert('Konto oprettet', 'Log ind med din nye bruger for at fortsætte.')
		router.replace('/login')
	}

	return (
		<ImageBackground
			source={require('../../../assets/images/login_background.png')}
			style={styles.container}
			resizeMode="cover"
		>
			<View style={styles.overlay} />
			<View style={styles.sheet}>
				<Pressable style={styles.backButton} onPress={() => (step === 1 ? router.back() : setStep(1))}>
					<Ionicons name='chevron-back' size={26} color='#7b7b7b' />
				</Pressable>

				<Text style={styles.title}>Sign Up</Text>

				{step === 1 ? (
					<>
						<View style={styles.inputRow}>
							<Ionicons name='mail-outline' size={20} color='#8c8c8c' style={styles.inputLeftIcon} />
							<TextInput
								placeholder="Enter your email"
								autoCapitalize="none"
								keyboardType="email-address"
								value={email}
								onChangeText={setEmail}
								style={styles.inputField}
								placeholderTextColor="#7f7f7f"
							/>
						</View>

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
						<View style={styles.inputRow}>
							<Ionicons name='lock-closed-outline' size={20} color='#8c8c8c' style={styles.inputLeftIcon} />
							<TextInput
								placeholder="Enter Password (8+ characters, 1 uppercase, 1 number)"
								autoCapitalize="none"
								secureTextEntry={!showPassword}
								value={password}
								onChangeText={setPassword}
								style={styles.inputField}
								placeholderTextColor="#7f7f7f"
							/>
							<Pressable style={styles.inputRightButton} onPress={() => setShowPassword((prev) => !prev)}>
								<Ionicons name={showPassword ? 'eye-outline' : 'eye-off-outline'} size={20} color='#8c8c8c' />
							</Pressable>
						</View>

						<View style={styles.inputRow}>
							<Ionicons name='lock-closed-outline' size={20} color='#8c8c8c' style={styles.inputLeftIcon} />
							<TextInput
								placeholder="Confirm password"
								autoCapitalize="none"
								secureTextEntry={!showConfirmPassword}
								value={confirmPassword}
								onChangeText={setConfirmPassword}
								style={styles.inputField}
								placeholderTextColor="#7f7f7f"
							/>
							<Pressable style={styles.inputRightButton} onPress={() => setShowConfirmPassword((prev) => !prev)}>
								<Ionicons name={showConfirmPassword ? 'eye-outline' : 'eye-off-outline'} size={20} color='#8c8c8c' />
							</Pressable>
						</View>

						<View style={styles.inputRow}>
							<Ionicons name='person-outline' size={20} color='#8c8c8c' style={styles.inputLeftIcon} />
							<TextInput
								placeholder="Enter a Username"
								autoCapitalize="none"
								value={username}
								onChangeText={setUsername}
								style={styles.inputField}
								placeholderTextColor="#7f7f7f"
							/>
						</View>

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
		justifyContent: 'flex-end',
	},
	overlay: {
		...StyleSheet.absoluteFillObject,
		backgroundColor: 'rgba(22, 38, 19, 0.30)',
	},
	sheet: {
		marginTop: '56%',
		
		backgroundColor: '#f7f7f7',
		borderTopLeftRadius: 36,
		borderTopRightRadius: 36,
		paddingHorizontal: 24,
		paddingTop: 28,
		paddingBottom: 64,
	},
	backButton: {
		position: 'absolute',
		top: 26,
		left: 18,
		width: 36,
		height: 36,
		borderRadius: 18,
		alignItems: 'center',
		justifyContent: 'center',
		zIndex: 2,
	},
	title: {
		fontSize: 24,
		fontFamily: 'RethinkSans_600SemiBold',
		color: '#101010',
		marginBottom: 18,
		justifyContent: 'center',
		alignItems: 'center',
		alignSelf: 'center',
	},
	inputRow: {
		borderWidth: 1,
		borderColor: '#d1d1d1',
		borderRadius: 24,
		minHeight: 45,
		backgroundColor: '#f1f1f1',
		marginBottom: 12,
		paddingHorizontal: 12,
		flexDirection: 'row',
		alignItems: 'center',
	},
	inputLeftIcon: {
		marginRight: 8,
	},
	inputField: {
		flex: 1,
		fontFamily: 'RethinkSans_400Regular',
		fontSize: 12,
	},
	inputRightButton: {
		width: 28,
		height: 28,
		alignItems: 'center',
		justifyContent: 'center',
		marginLeft: 8,
	},
	primaryButton: {
		marginTop: 8,
		minHeight: 45,
		borderRadius: 999,
		backgroundColor: '#5e7f56',
		justifyContent: 'center',
		alignItems: 'center',
	},
	primaryButtonText: {
		color: '#ffffff',
		fontSize: 18,
		fontFamily: 'RethinkSans_600SemiBold',
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
		fontFamily: 'RethinkSans_600SemiBold',
	},
})
