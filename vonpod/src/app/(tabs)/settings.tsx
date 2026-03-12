import { View, Text, Button, Alert } from "react-native";
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
      <Text style={{ fontSize: 24, fontWeight: "bold", paddingTop: 100}}>Settings</Text>
      <View style={{ marginTop: 24 }}>
        <Button
          title={loading ? "Logger ud..." : "Log ud"}
          onPress={handleSignOut}
          disabled={loading}
        />
      </View>
    </View>
  )
} 

export default Settings;