import { Stack } from "expo-router";
import { AuthProvider } from "../providers/AuthProvider";
import { SpecialGothicExpandedOne_400Regular, useFonts } from '@expo-google-fonts/special-gothic-expanded-one';
import { RethinkSans_400Regular } from '@expo-google-fonts/rethink-sans';
import { RethinkSans_500Medium } from "@expo-google-fonts/rethink-sans";
import { Mitr_500Medium } from '@expo-google-fonts/mitr';
import { Montserrat_400Regular } from '@expo-google-fonts/montserrat';
import { Montserrat_600SemiBold } from '@expo-google-fonts/montserrat';

export default function RootLayout() {
  const [fontsLoaded] = useFonts({
    SpecialGothicExpandedOne_400Regular,
    RethinkSans_400Regular,
    RethinkSans_500Medium,
    Mitr_500Medium,
    Montserrat_400Regular,
    Montserrat_600SemiBold,
  });

  if (!fontsLoaded) {
    return null;
  }

  return (
    <AuthProvider>
      <Stack screenOptions={{ headerShown: false }} />
    </AuthProvider>
  );
}