import type { MetaPacket } from '../../lib/imuPackets'
import type { StoredImuEvent } from '../../lib/imuStorage'

const MICROSECONDS_TO_MILLISECONDS = 1000

export type TempoResult = {
	ratio: number
	ratioLabel: string
	backswingMs: number
	downswingMs: number
}

function isPositiveFinite(value: number): boolean {
	return Number.isFinite(value) && value > 0
}

export function calculateTempoFromMeta(meta: MetaPacket): TempoResult | null {
	const { backswingStartUs, forwardStartUs, impactUs } = meta

	if (!isPositiveFinite(backswingStartUs) || !isPositiveFinite(forwardStartUs) || !isPositiveFinite(impactUs)) {
		return null
	}

	if (!(backswingStartUs < forwardStartUs && forwardStartUs < impactUs)) {
		return null
	}

	const backswingUs = forwardStartUs - backswingStartUs
	const downswingUs = impactUs - forwardStartUs

	if (!isPositiveFinite(backswingUs) || !isPositiveFinite(downswingUs)) {
		return null
	}

	const ratio = backswingUs / downswingUs
	if (!Number.isFinite(ratio) || ratio <= 0) {
		return null
	}

	return {
		ratio,
		ratioLabel: `${ratio.toFixed(2)}:1`,
		backswingMs: backswingUs / MICROSECONDS_TO_MILLISECONDS,
		downswingMs: downswingUs / MICROSECONDS_TO_MILLISECONDS,
	}
}

export function getLatestValidTempo(events: StoredImuEvent[]): TempoResult | null {
	const sortedByNewest = [...events].sort(
		(a, b) => new Date(b.savedAt).getTime() - new Date(a.savedAt).getTime()
	)

	for (const event of sortedByNewest) {
		if (!event.meta) {
			continue
		}

		const tempo = calculateTempoFromMeta(event.meta)
		if (tempo) {
			return tempo
		}
	}

	return null
}
