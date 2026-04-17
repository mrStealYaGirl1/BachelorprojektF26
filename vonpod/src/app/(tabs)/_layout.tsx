import { Redirect, Tabs } from "expo-router";
import { ActivityIndicator, View, Text, Dimensions, Alert, Pressable } from "react-native";
import React from "react";
import Ionicons from '@expo/vector-icons/Ionicons';
import { useRouter } from 'expo-router';
import { useAuth } from "../../providers/AuthProvider";
import { useTraining } from '../../providers/TrainingProvider';

const { width } = Dimensions.get("window");


export default function TabsLayout() {
  const { session, loading } = useAuth();
  const router = useRouter();
  const { isTraining, startTraining, stopTraining } = useTraining();

  const handleTrainingButtonPress = async () => {
    try {
      if (!isTraining) {
        await startTraining();
        router.push('/start-training');
        return;
      }

      router.push('/start-training');
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Kunne ikke opdatere traeningsstatus';
      Alert.alert('Fejl', message);
    }
  };

  const handleTrainingButtonLongPress = async () => {
    if (!isTraining) {
      return;
    }

    Alert.alert('Stop traening?', 'Hold op med at registrere denne session?', [
      {
        text: 'Annuller',
        style: 'cancel',
      },
      {
        text: 'Stop',
        style: 'destructive',
        onPress: async () => {
          try {
            await stopTraining();
            router.push('/activities');
          } catch (error) {
            const message =
              error instanceof Error ? error.message : 'Kunne ikke stoppe traening';
            Alert.alert('Fejl', message);
          }
        },
      },
    ]);
  };

  if (loading) {
    return (
      <View style={{ flex: 1, justifyContent: 'center', alignItems: 'center' }}>
        <ActivityIndicator />
      </View>
    );
  }

  if (!session) {
    return <Redirect href="/(auth)/welcome" />;
  }
  
  return (
    <Tabs screenOptions={{
      headerShown: false,
      tabBarShowLabel: false,
      tabBarStyle: {
        position: 'absolute',
        left: 16,
        right: 16,
        height: 90,
        elevation: 0,
        backgroundColor: '#fff',
        shadowColor: '#000',
        shadowOpacity: 0.12,
        shadowRadius: 12,
        shadowOffset: { width: 0, height: 5 },
        alignItems: 'center',
        justifyContent: 'center',
        paddingTop: 4,
      }
    }}>
      <Tabs.Screen 
        name="home" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              alignItems: "center", 
              paddingTop: 10,
              width: width/5
              }}>
                <Ionicons
                  name={focused ? "home" : "home-outline"}
                  color={focused ? "#4d7d48" : "#888"}
                  size={24}
                />
                <Text style={{ 
                  color: focused ? "#000" : "#888", 
                  fontSize: 12, 
                  marginTop: 4,
                }}>
                  Home
                </Text>
              </View>
            ),
          }} 
        />
        <Tabs.Screen 
        name="activities" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              alignItems: "center", 
              paddingTop: 10,
              width: width/5
              }}>
                <Ionicons
                  name={focused ? "golf" : "golf-outline"}
                  color={focused ? "#4d7d48" : "#888"}
                  size={24}
                />
                <Text style={{ 
                  color: focused ? "#000" : "#888", 
                  fontSize: 12, 
                  marginTop: 4,
                }}>
                  Activities
                </Text>
              </View>
            ),
          }} 
        />
        <Tabs.Screen 
        name="start-training" 
        options={{
          tabBarButton: () => (
            <Pressable
              onPress={handleTrainingButtonPress}
              onLongPress={handleTrainingButtonLongPress}
              style={{
                height: 70,
                width: 70,
                alignItems: 'center',
                justifyContent: 'center',
                alignSelf: 'center',
                borderRadius: 99999,
                marginTop: -11,
                paddingLeft: isTraining ? 0 : 3,
                borderWidth: 3,
                borderColor: '#ffffff',
                shadowColor: '#000000',
                shadowOpacity: 0.18,
                shadowRadius: 4,
                shadowOffset: { width: 0, height: 2 },
                elevation: 4,
                backgroundColor: isTraining ? '#b03a3a' : '#4d7d48',
              }}
            >
              <Ionicons
                name={isTraining ? 'stop' : 'play'}
                color="#fff"
                size={34}
              />
            </Pressable>
          ),
          }} 
        />
        <Tabs.Screen 
        name="profile" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              alignItems: "center", 
              paddingTop: 10,
              width: width/5
              }}>
                <Ionicons
                  name={focused ? "person" : "person-outline"}
                  color={focused ? "#4d7d48" : "#888"}
                  size={24}
                />
                <Text style={{ 
                  color: focused ? "#000" : "#888", 
                  fontSize: 12, 
                  marginTop: 4,
                }}>
                  Profile
                </Text>
              </View>
            ),
          }} 
        />
        <Tabs.Screen 
        name="settings" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              alignItems: "center", 
              paddingTop: 10,
              width: width/5
              }}>
                <Ionicons
                  name={focused ? "settings" : "settings-outline"}
                  color={focused ? "#4d7d48" : "#888"}
                  size={24}
                />
                <Text style={{ 
                  color: focused ? "#000" : "#888", 
                  fontSize: 12, 
                  marginTop: 4,
                }}>
                  Settings
                </Text>
              </View>
            ),
          }} 
        />
        <Tabs.Screen
        name="activity-details"
        options={{
          href: null,
          headerShown: false,
        }}
        />
    </Tabs>
  );
}