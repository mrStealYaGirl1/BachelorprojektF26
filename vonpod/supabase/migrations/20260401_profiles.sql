create table if not exists public.profiles (
  id uuid references auth.users(id) on delete cascade not null primary key,
  updated_at timestamptz,
  first_name text,
  last_name text,
  username text unique not null,
  constraint username_length check (char_length(username) >= 3)
);

alter table public.profiles enable row level security;

drop policy if exists "Users can view own profile." on public.profiles;
create policy "Users can view own profile."
on public.profiles
for select
using ((select auth.uid()) = id);

drop policy if exists "Users can insert their own profile." on public.profiles;
create policy "Users can insert their own profile."
on public.profiles
for insert
with check ((select auth.uid()) = id);

drop policy if exists "Users can update own profile." on public.profiles;
create policy "Users can update own profile."
on public.profiles
for update
using ((select auth.uid()) = id)
with check ((select auth.uid()) = id);

create or replace function public.handle_new_user()
returns trigger
set search_path = ''
as $$
begin
  insert into public.profiles (id, username)
  values (new.id, new.raw_user_meta_data->>'username');
  return new;
end;
$$ language plpgsql security definer;

drop trigger if exists on_auth_user_created on auth.users;
create trigger on_auth_user_created
  after insert on auth.users
  for each row execute procedure public.handle_new_user();
