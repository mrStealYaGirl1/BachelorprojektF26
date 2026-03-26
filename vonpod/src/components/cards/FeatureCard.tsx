import { View, Text, StyleSheet, Pressable } from 'react-native';

type FeatureCardProps = {
  title: string;
  value: string;
  unitLabel: string;
  onPress?: () => void;
};

export default function FeatureCard({
  title,
  value,
  unitLabel,
  onPress,
}: FeatureCardProps) {
  return (
    <Pressable style={styles.card} onPress={onPress}>
      <Text style={styles.title} numberOfLines={1} adjustsFontSizeToFit minimumFontScale={0.75}>
        {title}
      </Text>

      <Text style={styles.value} numberOfLines={1} adjustsFontSizeToFit minimumFontScale={0.75}>{value}</Text>

      <Text style={styles.unitLabel}>{unitLabel}</Text>
    </Pressable>
  );
}

const styles = StyleSheet.create({
  card: {
    width: '44%',
    aspectRatio: 1,
    backgroundColor: '#E9E5E4',
    borderRadius: 32,
    boxShadow: 'rgba(100, 100, 111, 0.35) 0px 1px 8px 0px',
    paddingHorizontal: 20,
    paddingTop: 26,
    paddingBottom: 22,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 10,
    elevation: 3,
    margin: 10,
    justifyContent: 'space-between',
  },
  title: {
    fontSize: 16,
    lineHeight: 20,
    fontWeight: '600',
    color: '#111',
    textAlign: 'center',
    fontFamily: 'RethinkSans_500Medium',
  },
  value: {
    fontSize: 50,
    lineHeight: 62,
    fontWeight: '800',
    color: '#000',
    textAlign: 'center',
    marginTop: 12,
    fontFamily: 'SpecialGothicExpandedOne_400Regular',
  },
  unitLabel: {
    fontSize: 14,
    lineHeight: 18,
    fontWeight: '400',
    color: '#111',
    textAlign: 'center',
    marginTop: 8,
    fontFamily: 'RethinkSans_400Regular',
  },
});