import { Stack } from "expo-router";
import { AuthProvider } from "../providers/AuthProvider";
import { SpecialGothicExpandedOne_400Regular, useFonts } from '@expo-google-fonts/special-gothic-expanded-one';
import { RethinkSans_400Regular } from '@expo-google-fonts/rethink-sans';
import { RethinkSans_500Medium } from "@expo-google-fonts/rethink-sans";
import { RethinkSans_600SemiBold } from "@expo-google-fonts/rethink-sans";
import { Mitr_500Medium } from '@expo-google-fonts/mitr';
import { Mitr_600SemiBold } from "@expo-google-fonts/mitr";
import { Montserrat_400Regular } from '@expo-google-fonts/montserrat';
import { Montserrat_600SemiBold } from '@expo-google-fonts/montserrat';
import { RethinkSans_800ExtraBold } from "@expo-google-fonts/rethink-sans";
import { TrainingProvider } from '../providers/TrainingProvider';

export default function RootLayout() {
  const [fontsLoaded] = useFonts({
    SpecialGothicExpandedOne_400Regular,
    RethinkSans_400Regular,
    RethinkSans_500Medium,
    RethinkSans_600SemiBold,
    RethinkSans_800ExtraBold,
    Mitr_500Medium,
    Mitr_600SemiBold,
    Montserrat_400Regular,
    Montserrat_600SemiBold,
  });

  if (!fontsLoaded) {
    return null;
  }

  return (
    <AuthProvider>
      <TrainingProvider>
        <Stack screenOptions={{ headerShown: false }} />
      </TrainingProvider>
    </AuthProvider>
  );
}