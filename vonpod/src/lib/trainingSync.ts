import { supabase } from './supabase'
import type { StoredTrainingSession } from './trainingStorage'

export async function syncTrainingSessionToSupabase(
  session: StoredTrainingSession
): Promise<void> {
  if (!session.userId) {
    return
  }

  const { error: sessionError } = await supabase.from('training_sessions').upsert(
    {
      id: session.id,
      user_id: session.userId,
      started_at: session.startedAt,
      ended_at: session.endedAt,
      duration_seconds: session.durationSeconds,
      putts_count: session.puttsCount,
      created_at: session.createdAt,
    },
    { onConflict: 'id' }
  )

  if (sessionError) {
    throw sessionError
  }

  if (session.puttEventIds.length === 0) {
    return
  }

  const payload = session.puttEventIds.map((eventId, index) => ({
    session_id: session.id,
    putt_index: index + 1,
    event_id: eventId,
    user_id: session.userId,
  }))

  const { error: puttsError } = await supabase
    .from('training_session_putts')
    .upsert(payload, { onConflict: 'session_id,putt_index' })

  if (puttsError) {
    throw puttsError
  }
}
