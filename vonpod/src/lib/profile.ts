import type { User } from '@supabase/supabase-js'
import { supabase } from './supabase'

export type ProfileRow = {
  id: string
  username: string
  first_name: string | null
  last_name: string | null
  updated_at: string | null
}

function sanitizeUsername(value: string): string {
  return value.trim().toLowerCase().replace(/\s+/g, '_')
}

export function getProfileUsernameFallback(user: User): string {
  const metadataUsername =
    typeof user.user_metadata?.username === 'string' && user.user_metadata.username.trim().length > 0
      ? sanitizeUsername(user.user_metadata.username)
      : null

  if (metadataUsername && metadataUsername.length >= 3) {
    return metadataUsername
  }

  const emailPrefix = user.email?.split('@')[0]
  if (emailPrefix) {
    const sanitizedEmailPrefix = sanitizeUsername(emailPrefix)
    if (sanitizedEmailPrefix.length >= 3) {
      return sanitizedEmailPrefix
    }
  }

  return `user_${user.id.slice(0, 8)}`
}

export async function fetchOwnProfile(userId: string): Promise<ProfileRow | null> {
  const { data, error } = await supabase
    .from('profiles')
    .select('id, username, first_name, last_name, updated_at')
    .eq('id', userId)
    .maybeSingle()

  if (error) {
    throw error
  }

  return data
}

export async function upsertOwnProfileNames(params: {
  user: User
  username?: string
  firstName: string
  lastName: string
}): Promise<ProfileRow> {
  const resolvedUsername =
    params.username && params.username.trim().length >= 3
      ? params.username.trim()
      : getProfileUsernameFallback(params.user)

  const { data, error } = await supabase
    .from('profiles')
    .upsert(
      {
        id: params.user.id,
        username: resolvedUsername,
        first_name: params.firstName.trim() || null,
        last_name: params.lastName.trim() || null,
        updated_at: new Date().toISOString(),
      },
      { onConflict: 'id' }
    )
    .select('id, username, first_name, last_name, updated_at')
    .single()

  if (error) {
    throw error
  }

  return data
}
