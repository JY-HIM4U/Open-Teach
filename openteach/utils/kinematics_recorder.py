"""
Lightweight per-process CSV recorder for teleop kinematics.

Each teleop operator runs in its OWN process, so start/stop can't come from a
single keypress in one process. Instead a tiny controller (record_ctl.py)
writes a *sentinel file* containing a session id while recording is active;
every operator polls that file and opens/closes its own CSV in lockstep. All
files from one session share the same <session_id> label, so the hand and arm
CSVs line up (and every row also carries its own epoch timestamp for fine
alignment).

Design goals: no extra config/ports, survives Ctrl-C (rows are flushed), and
records in BOTH dry-run and real mode (same call site).
"""

import csv
import os
import time


DEFAULT_DIR = 'logs/recordings'
SENTINEL_NAME = '.recording'


def sentinel_path(ctrl_dir=DEFAULT_DIR):
    return os.path.join(ctrl_dir, SENTINEL_NAME)


def start_session(label, ctrl_dir=DEFAULT_DIR):
    """Begin a recording session (all recorders polling this sentinel open a
    new CSV labelled <label>). Written atomically so no one reads a half id."""
    os.makedirs(ctrl_dir, exist_ok=True)
    path = sentinel_path(ctrl_dir)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(str(label))
    os.replace(tmp, path)


def stop_session(ctrl_dir=DEFAULT_DIR):
    """End the current recording session (recorders close their CSVs)."""
    try:
        os.remove(sentinel_path(ctrl_dir))
    except FileNotFoundError:
        pass


class SegmentRecorder:
    """One CSV stream for one operator. Call maybe_log(row) every loop cycle;
    it no-ops until the controller starts a session, then writes a row per call
    until the session stops."""

    def __init__(self, name, fieldnames, ctrl_dir=DEFAULT_DIR, poll_every=6):
        self.name = name
        self.fieldnames = ['t_epoch', 't_rel'] + list(fieldnames)
        self.ctrl_dir = ctrl_dir
        self.sentinel = sentinel_path(ctrl_dir)
        self.poll_every = max(1, poll_every)

        self._session = None      # session id we're currently writing under
        self._fh = None
        self._writer = None
        self._t0 = None
        self._poll_counter = 0
        self._active = False      # cached sentinel state between polls
        os.makedirs(ctrl_dir, exist_ok=True)

    def _read_session(self):
        try:
            with open(self.sentinel) as f:
                sid = f.read().strip()
            return sid or None
        except (FileNotFoundError, OSError):
            return None

    def maybe_log(self, row):
        """row: dict of the fieldnames (minus timestamps). Missing keys are
        left blank; extras are ignored."""
        # Throttle the filesystem poll; logging itself stays at full rate.
        self._poll_counter += 1
        if self._poll_counter % self.poll_every == 0:
            sid = self._read_session()
            if sid != self._session:
                self._close()
                self._session = sid
                if sid:
                    self._open(sid)

        if self._writer is None or row is None:
            return
        now = time.time()
        full = {'t_epoch': round(now, 4), 't_rel': round(now - self._t0, 4)}
        full.update(row)
        self._writer.writerow(full)
        self._fh.flush()

    def is_active(self):
        """True while a recording session is open (a CSV is being written)."""
        return self._writer is not None

    def _open(self, sid):
        path = os.path.join(self.ctrl_dir, '{}_{}.csv'.format(self.name, sid))
        self._fh = open(path, 'w', newline='')
        self._writer = csv.DictWriter(
            self._fh, fieldnames=self.fieldnames, extrasaction='ignore')
        self._writer.writeheader()
        self._t0 = time.time()
        print('[REC] {}: recording -> {}'.format(self.name, path))

    def _close(self):
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            finally:
                print('[REC] {}: stopped (session {})'.format(
                    self.name, self._session))
        self._fh = None
        self._writer = None

    def close(self):
        self._close()
