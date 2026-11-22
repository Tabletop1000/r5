import type { Units } from '../display/units';
import type { PlanJobResponse } from '../ros/topics';
import type { Limits, SettingsForm } from './settings';

/** Area Spray setup in progress, kept while the user visits other pages. */
export interface SetupDraft {
  fieldId: string;
  /** The settings form, filled in for formFieldId, in formUnits. */
  form: SettingsForm | null;
  formFieldId: string;
  formUnits: Units;
  limits: Limits | null;
  planResult: PlanJobResponse | null;
}

// Module state rather than browser storage: it lasts until the page is reloaded, so an old plan
// from another day never comes back.
let saved: SetupDraft | null = null;

export function loadSetupDraft(): SetupDraft | null {
  return saved;
}

export function saveSetupDraft(draft: SetupDraft): void {
  saved = draft;
}

/** After a job starts, setup begins afresh next time. */
export function clearSetupDraft(): void {
  saved = null;
}
