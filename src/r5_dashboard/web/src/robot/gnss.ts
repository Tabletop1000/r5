import type { NavPvt, NavSatFix } from '../ros/topics';

export type FixQuality = 'none' | 'gps' | 'dgps' | 'rtk-float' | 'rtk-fixed';

export const FIX_LABEL: Record<FixQuality, string> = {
  none: 'No fix',
  gps: '3D fix',
  dgps: 'Corrected (SBAS)',
  'rtk-float': 'RTK float',
  'rtk-fixed': 'RTK fixed',
};

export interface GnssSummary {
  quality: FixQuality;
  /** Horizontal accuracy, m, if known. */
  accuracyM: number | null;
  satellites: number | null;
  latitude: number | null;
  longitude: number | null;
}

// sensor_msgs/NavSatStatus
const STATUS_NO_FIX = -1;
const STATUS_SBAS_FIX = 1;
const STATUS_GBAS_FIX = 2;
const COVARIANCE_TYPE_UNKNOWN = 0;

/**
 * Combines /fix with the u-blox NavPVT message when it is available. NavSatFix alone can't tell
 * RTK float from RTK fixed, or count satellites.
 */
export function summarizeGnss(fix: NavSatFix | null, pvt: NavPvt | null): GnssSummary {
  let quality: FixQuality = 'none';
  if (fix && fix.status.status > STATUS_NO_FIX) {
    if (fix.status.status === STATUS_GBAS_FIX) quality = 'rtk-float';
    else if (fix.status.status === STATUS_SBAS_FIX) quality = 'dgps';
    else quality = 'gps';
  }
  if (pvt) {
    const carrier = (pvt.flags >> 6) & 0b11; // NavPVT flags carrSoln: 1 float, 2 fixed
    if (carrier === 2) quality = 'rtk-fixed';
    else if (carrier === 1) quality = 'rtk-float';
    else if (quality === 'rtk-float') quality = 'dgps';
  }

  let accuracyM: number | null = null;
  if (pvt && Number.isFinite(pvt.h_acc)) accuracyM = pvt.h_acc / 1000;
  else if (fix && fix.position_covariance_type !== COVARIANCE_TYPE_UNKNOWN) {
    const cov = fix.position_covariance;
    if (cov && Number.isFinite(cov[0]) && Number.isFinite(cov[4])) accuracyM = Math.sqrt((cov[0] + cov[4]) / 2);
  }

  const hasPosition = fix !== null && quality !== 'none' && Number.isFinite(fix.latitude) && Number.isFinite(fix.longitude);
  return {
    quality,
    accuracyM,
    satellites: pvt ? pvt.num_sv : null,
    latitude: hasPosition ? fix.latitude : null,
    longitude: hasPosition ? fix.longitude : null,
  };
}
