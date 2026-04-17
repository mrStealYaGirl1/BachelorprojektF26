create table if not exists public.training_sessions (
  id text primary key,
  user_id uuid not null references auth.users(id) on delete cascade,
  started_at timestamptz not null,
  ended_at timestamptz not null,
  duration_seconds integer not null check (duration_seconds >= 0),
  putts_count integer not null default 0 check (putts_count >= 0),
  created_at timestamptz not null default now()
);

create table if not exists public.training_session_putts (
  session_id text not null references public.training_sessions(id) on delete cascade,
  putt_index integer not null check (putt_index > 0),
  event_id text not null,
  user_id uuid not null references auth.users(id) on delete cascade,
  created_at timestamptz not null default now(),
  primary key (session_id, putt_index)
);

alter table public.training_sessions enable row level security;
alter table public.training_session_putts enable row level security;

drop policy if exists "Users can view own training sessions" on public.training_sessions;
create policy "Users can view own training sessions"
on public.training_sessions
for select
using ((select auth.uid()) = user_id);

drop policy if exists "Users can insert own training sessions" on public.training_sessions;
create policy "Users can insert own training sessions"
on public.training_sessions
for insert
with check ((select auth.uid()) = user_id);

drop policy if exists "Users can update own training sessions" on public.training_sessions;
create policy "Users can update own training sessions"
on public.training_sessions
for update
using ((select auth.uid()) = user_id)
with check ((select auth.uid()) = user_id);

drop policy if exists "Users can view own training putts" on public.training_session_putts;
create policy "Users can view own training putts"
on public.training_session_putts
for select
using ((select auth.uid()) = user_id);

drop policy if exists "Users can insert own training putts" on public.training_session_putts;
create policy "Users can insert own training putts"
on public.training_session_putts
for insert
with check ((select auth.uid()) = user_id);

drop policy if exists "Users can update own training putts" on public.training_session_putts;
create policy "Users can update own training putts"
on public.training_session_putts
for update
using ((select auth.uid()) = user_id)
with check ((select auth.uid()) = user_id);
