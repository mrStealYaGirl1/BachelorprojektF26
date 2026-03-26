import { View, Text, StyleSheet, ScrollView, Pressable } from 'react-native';
import FeatureCard from '../../components/cards/FeatureCard';

export default function StartTrainingScreen() {
  return (
    <View style={styles.screen}>
      <Text style={styles.logo}>vonpod</Text>

      <View style={styles.navigator}>
        <Pressable style={styles.arrowButton}>
          <Text style={styles.arrow}>‹</Text>
        </Pressable>

        <View style={styles.puttNumbers}>
          <Text style={styles.inactivePutt}>1</Text>
          <Text style={styles.inactivePutt}>2</Text>
          <Text style={styles.inactivePutt}>3</Text>
          <Text style={styles.inactivePutt}>4</Text>
          <Text style={styles.inactivePutt}>5</Text>
          <Text style={styles.inactivePutt}>6</Text>
          <Text style={styles.activePutt}>7</Text>
          <Text style={styles.inactivePutt}>8</Text>
          <Text style={styles.inactivePutt}>9</Text>
          <Text style={styles.inactivePutt}>10</Text>
        </View>

        <Pressable style={styles.arrowButton}>
          <Text style={styles.arrow}>›</Text>
        </Pressable>
      </View>

      <ScrollView
        contentContainerStyle={styles.scrollContent}
        showsVerticalScrollIndicator={false}
      >
        <View style={styles.grid}>
          <FeatureCard
            title="Tempo"
            value="2.2:1"
            unitLabel="ratio"
          />
          <FeatureCard
            title="Backswing Length"
            value="35"
            unitLabel="cm"
          />
          <FeatureCard
            title="Back Stroke Rotation"
            value="2.7"
            unitLabel="deg · open"
          />
          <FeatureCard
            title="Forward Stroke Rotation"
            value="2.6"
            unitLabel="deg · closed"
          />
          <FeatureCard
            title="Lie Impact Angle"
            value="0.7"
            unitLabel="deg · toe up"
          />
          <FeatureCard
            title="Face Angle at Impact"
            value="0.1"
            unitLabel="deg · open"
          />
        </View>

        <Pressable style={styles.reorderButton}>
          <Text style={styles.reorderText}>Reorder</Text>
          <Text style={styles.reorderIcon}>▦</Text>
        </Pressable>
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  screen: {
    flex: 1,
    backgroundColor: '#F4F3F1',
    paddingTop: 54,
    paddingHorizontal: 10,
  },
  logo: {
    fontSize: 28,
    fontFamily: 'Mitr_500Medium',
    textShadowColor: 'rgba(4, 4, 0, 0.35)',
    textShadowOffset: {width: -1, height: 1},
    textShadowRadius: 10,
    fontWeight: '800',
    textAlign: 'center',
    color: '#222',
    marginBottom: 4,
  },
  navigator: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  arrowButton: {
    width: 28,
    alignItems: 'center',
    justifyContent: 'center',
  },
  arrow: {
    fontSize: 34,
    color: '#111',
    fontWeight: '300',
    lineHeight: 34,
  },
  puttNumbers: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 18,
  },
  inactivePutt: {
    fontSize: 18,
    fontFamily: 'Montserrat_400Regular',
    color: '#C9C7C4',
    fontWeight: '400',
  },
  activePutt: {
    fontSize: 18,
    fontFamily: 'Montserrat_600SemiBold',
    color: '#111',
    fontWeight: '600',
  },
  scrollContent: {
    paddingBottom: 140,
  },
  grid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  reorderButton: {
    alignSelf: 'center',
    marginTop: 6,
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
    backgroundColor: '#ECE8E6',
    borderRadius: 999,
    paddingHorizontal: 18,
    paddingVertical: 8,
  },
  reorderText: {
    fontSize: 12,
    fontWeight: '500',
    color: '#111',
    fontFamily: 'RethinkSans_400Regular',
  },
  reorderIcon: {
    fontSize: 18,
    color: '#111',
  },
});