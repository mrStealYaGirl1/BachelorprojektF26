import { View, Text, Alert, Pressable, StyleSheet } from "react-native";
import React, { useState } from "react";
import { useAuth } from "../../providers/AuthProvider";

const Settings = () => {
  const { signOut } = useAuth();
  const [loading, setLoading] = useState(false);

  const handleSignOut = async () => {
    try {
      setLoading(true);
      await signOut();
    } catch (error) {
      const message = error instanceof Error ? error.message : "Kunne ikke logge ud";
      Alert.alert("Fejl", message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <View style={{ padding: 24 }}>
      <Text style={{ fontSize: 24, fontWeight: "bold", paddingTop: 100, paddingBottom: 24 }}>
        Settings
      </Text>

      <View style={styles.container}>
        <Pressable
          style={[styles.button, loading && styles.buttonDisabled]}
          onPress={handleSignOut}
          disabled={loading}
        >
          <Text style={styles.buttonText}>
            {loading ? "Logger ud..." : "Log ud"}
          </Text>
        </Pressable>
      </View>
    </View>
  )
} 

const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },

  button: {
    marginTop: 8,
    minHeight: 52,
    minWidth: 180,
    borderRadius: 999,
    backgroundColor: '#5e7f56',
    alignItems: 'center',
    justifyContent: 'center',
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
});

export default Settings;