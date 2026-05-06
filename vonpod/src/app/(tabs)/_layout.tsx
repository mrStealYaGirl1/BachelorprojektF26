import { Redirect, Tabs, usePathname, useRouter } from "expo-router";
import { ActivityIndicator, View, Text, Dimensions, Alert, Pressable, Animated, Easing } from "react-native";
import React from "react";
import Ionicons from '@expo/vector-icons/Ionicons';
import { useAuth } from "../../providers/AuthProvider";
import { useTraining } from '../../providers/TrainingProvider';

const { width } = Dimensions.get("window");


export default function TabsLayout() {
  const { session, loading } = useAuth();
  const router = useRouter();
  const pathname = usePathname();
  const { isTraining, startTraining, stopTraining } = useTraining();

  const isOnStartTrainingScreen =
    pathname.endsWith('/start-training') || pathname.endsWith('/(tabs)/start-training');

  const fade = React.useRef(new Animated.Value(0)).current

  React.useEffect(() => {
    // show a quick semi-opaque overlay then fade it out for a soft transition
    fade.setValue(0.6)
    Animated.timing(fade, {
      toValue: 0,
      duration: 200,
      easing: Easing.out(Easing.cubic),
      useNativeDriver: true,
    }).start()
  }, [pathname, fade])

  const confirmStopTraining = () => {
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

  const handleTrainingButtonPress = async () => {
    try {
      if (!isTraining) {
        await startTraining();
        router.push('/start-training');
        return;
      }

      if (isOnStartTrainingScreen) {
        confirmStopTraining();
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

    confirmStopTraining();
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
    <View style={{ flex: 1, position: 'relative' }}>
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
          animation: 'none',
        }}
        />
      </Tabs>

      {/* overlay used to create a minimal soft transition when tabs change */}
      <Animated.View
        pointerEvents="none"
        style={{
          position: 'absolute',
          left: 0,
          right: 0,
          top: 0,
          // leave more space at the bottom so the large center button is not overlapped
          bottom: 140,
          backgroundColor: '#fff',
          opacity: fade,
        }}
      />
    </View>
  );
}