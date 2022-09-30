/*  SPDX-License-Identifier: GPL-3.0-or-later

    Copyright (C) 2022  Povilas Kanapickas <povilas@radix.lt>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

package com.p12tic.mobisane;

import android.view.Surface;

public class NativeAppManager {
    public native void setPreviewSurface(Surface surface);
    public native void notifyResourcesReady(String rootPath);

    public native void startNewSession(String destPath);
    public native void startAnalysis();
    public native boolean isAnalysisFinished();
    public native boolean isAnalysisSuccess();

    public native String getCurrentStatus();

    // Returns NaN if no progress is currently recorded
    public native double getCurrentProgress();

    static {
        System.loadLibrary("mobisaneapp");
    }
}
