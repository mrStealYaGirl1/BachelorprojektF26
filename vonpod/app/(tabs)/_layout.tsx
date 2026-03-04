import { Tabs } from "expo-router";
import { View, Text, Dimensions } from "react-native";
import React from "react";
import Ionicons from '@expo/vector-icons/Ionicons';

const { width, height } = Dimensions.get("window");


export default function TabsLayout() {
  
  return (
    <Tabs screenOptions={{
      headerShown: false,
      tabBarShowLabel: false,
      tabBarStyle: {
        position: 'absolute',
         
        left: 16,
        right: 16,
        height: 99,
        elevation: 0,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
      }
    }}>
      <Tabs.Screen 
        name="index" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              alignItems: "center", 
              paddingTop: 10,
              width: width/5
              }}>
                <Ionicons
                  name={focused ? "home" : "home-outline"}
                  color={focused ? "#59008c" : "#888"}
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
                  color={focused ? "#59008c" : "#888"}
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
        name="startTraining" 
        options={{
          tabBarIcon: ({ focused }) => (
            <View style={{ 
              height: 65,
              width: 65,
              alignItems: "center", 
              justifyContent: "center",
              paddingLeft: 5,
              borderRadius: 99999,
              backgroundColor: "#4d7d48",
              }}>
                <Ionicons
                  name={focused ? "play" : "play-outline"}
                  color={focused ? "#fff" : "#fff"}
                  size={36}
                />
              </View>
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
                  color={focused ? "#59008c" : "#888"}
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
                  color={focused ? "#59008c" : "#888"}
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
    </Tabs>
  );
}