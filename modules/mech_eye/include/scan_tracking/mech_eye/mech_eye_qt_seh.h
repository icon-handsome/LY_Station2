#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// Copy a QString under SEH. Corrupted Qt strings (null d-pointer) return empty
/// instead of taking down the process — matches field dump Qt5Core+0x15d3.
QString safeCopyQString(const QString& src);

/// Field-wise safe copy of camera snapshot strings.
CameraInfoSnapshot safeCopyCameraInfo(const CameraInfoSnapshot& src);

/// Best-effort reset when snapshot strings may already be corrupt: destroy under
/// SEH then placement-new a fresh empty snapshot.
void forceResetCameraInfo(CameraInfoSnapshot* info);

/// Destroy-under-SEH then move-assign a fresh snapshot (avoids dtor AV on corrupt old).
void replaceCameraInfo(CameraInfoSnapshot* dst, CameraInfoSnapshot src);

}  // namespace scan_tracking::mech_eye
