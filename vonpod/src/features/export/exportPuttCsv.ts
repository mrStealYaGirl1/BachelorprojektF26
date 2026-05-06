import JSZip from 'jszip'
import * as FileSystem from 'expo-file-system/legacy'
import { Share } from 'react-native'
import type { MetaPacket } from '../../lib/imuPackets'
import type { StoredImuEvent } from '../../lib/imuStorage'

function sanitizeFileName(value: string): string {
  return value
    .trim()
    .toLowerCase()
    .replace(/\s+/g, '_')
    .replace(/[^a-z0-9_-]/g, '')
}

function csvEscape(value: string | number | boolean | null | undefined): string {
  if (value === null || value === undefined) {
    return ''
  }

  const text = String(value)
  if (text.includes(',') || text.includes('"') || text.includes('\n')) {
    return `"${text.replace(/"/g, '""')}"`
  }

  return text
}

function buildDataCsv(event: StoredImuEvent): string {
  const header = ['seq', 'ts_ms', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'accel_g', 'gyro_dps']
  const rows = [header.join(',')]

  const sortedSamples = [...event.samples].sort((leftSample, rightSample) => leftSample.seq - rightSample.seq)
  for (const sample of sortedSamples) {
    rows.push(
      [
        sample.seq,
        sample.tsMs,
        sample.ax,
        sample.ay,
        sample.az,
        sample.gx,
        sample.gy,
        sample.gz,
        sample.accelG,
        sample.gyroDps,
      ]
        .map((value) => csvEscape(value))
        .join(',')
    )
  }

  return `${rows.join('\n')}\n`
}

function buildMetaEntries(event: StoredImuEvent, meta?: MetaPacket): Array<[string, string | number | boolean]> {
  const entries: Array<[string, string | number | boolean]> = [
    ['id', event.id],
    ['local_event_id', event.localEventId ?? ''],
    ['event_id', event.eventId],
    ['saved_at', event.savedAt],
    ['sample_count', event.sampleCount],
    ['has_meta', Boolean(meta)],
  ]

  if (!meta) {
    return entries
  }

  entries.push(
    ['meta.swing_id', meta.swingId],
    ['meta.sample_rate_hz', meta.sampleRateHz],
    ['meta.total_samples', meta.totalSamples],
    ['meta.pre_samples', meta.preSamples],
    ['meta.post_samples', meta.postSamples],
    ['meta.impact_index_in_event', meta.impactIndexInEvent],
    ['meta.address_start_us', meta.addressStartUs],
    ['meta.backswing_start_us', meta.backswingStartUs],
    ['meta.forward_start_us', meta.forwardStartUs],
    ['meta.impact_us', meta.impactUs],
    ['meta.follow_start_us', meta.followStartUs],
    ['meta.end_us', meta.endUs],
    ['meta.event_start_us', meta.eventStartUs],
    ['meta.event_end_us', meta.eventEndUs]
  )

  return entries
}

function buildMetaCsv(event: StoredImuEvent): string {
  const rows = ['key,value']
  for (const [key, value] of buildMetaEntries(event, event.meta)) {
    rows.push(`${csvEscape(key)},${csvEscape(value)}`)
  }

  return `${rows.join('\n')}\n`
}

export async function createPuttExportZip(sessionTitle: string, selectedPutts: StoredImuEvent[]): Promise<string> {
  if (selectedPutts.length === 0) {
    throw new Error('Ingen putts valgt til export')
  }

  const zip = new JSZip()

  selectedPutts.forEach((putt, index) => {
    const puttLabel = putt.localEventId ?? index + 1
    const folder = zip.folder(`putt_${puttLabel}`)
    if (!folder) {
      return
    }

    folder.file('data.csv', buildDataCsv(putt))
    folder.file('meta.csv', buildMetaCsv(putt))
  })

  const base64Zip = await zip.generateAsync({
    type: 'base64',
    compression: 'DEFLATE',
    compressionOptions: { level: 6 },
  })

  const rootDir = FileSystem.cacheDirectory ?? FileSystem.documentDirectory
  if (!rootDir) {
    throw new Error('Kunne ikke finde midlertidig mappe')
  }

  const safeSessionTitle = sanitizeFileName(sessionTitle || 'session') || 'session'
  const timestamp = new Date().toISOString().replace(/[:.]/g, '-')
  const zipUri = `${rootDir}putt_export_${safeSessionTitle}_${timestamp}.zip`

  await FileSystem.writeAsStringAsync(zipUri, base64Zip, {
    encoding: FileSystem.EncodingType.Base64,
  })

  return zipUri
}

export async function sharePuttExportZip(zipUri: string): Promise<void> {
  try {
    const fileName = zipUri.split('/').pop() || 'putt_export.zip'
    await Share.share({
      url: zipUri, // iOS accepts file:// URIs
      title: 'Export putts',
      message: 'Her er din putt-export ZIP fil',
    })
  } catch (error) {
    if (error instanceof Error && error.message === 'User did not share') {
      // User cancelled share sheet
      return
    }
    throw error
  }
}
