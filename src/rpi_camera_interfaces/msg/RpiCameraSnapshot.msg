# Full camera control snapshot (typed + JSON mirror of the HTTP /api/cameras body).
string backend
string webrtc
string error
string[] live
string[] restart
RpiCameraItem[] items
RpiCameraZoomPreset[] zoom
RpiCameraState[] state
RpiCameraStorage storage
RpiCameraRecordMeta record
string snapshot_json
