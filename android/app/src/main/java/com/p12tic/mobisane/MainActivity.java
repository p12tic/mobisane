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

import android.Manifest;
import android.content.pm.PackageManager;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.util.Size;
import android.view.Gravity;
import android.view.Surface;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.material.floatingactionbutton.FloatingActionButton;
import com.p12tic.mobisane.databinding.ActivityMainBinding;

import android.view.Menu;
import android.view.MenuItem;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;

public class MainActivity extends AppCompatActivity
        implements TextureView.SurfaceTextureListener,
        ActivityCompat.OnRequestPermissionsResultCallback {
    static final int PERMISSION_REQUEST_CAMERA = 10;

    private NativeCamera nativeCamera = new NativeCamera();

    private ActivityMainBinding binding;

    private TextureView cameraView;
    private Surface surface = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        setSupportActionBar(binding.toolbar);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraView = (TextureView) findViewById(R.id.cameraView);
        cameraView.setSurfaceTextureListener(this);

        FloatingActionButton takePhotoButton = (FloatingActionButton) findViewById(R.id.takePhoto);
        takePhotoButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (nativeCamera.isOpen()) {
                    nativeCamera.captureImage();
                }
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture,
                                          int width, int height) {
        if (!nativeCamera.isOpen()) {
            return;
        }

        Size cameraSize = nativeCamera.getBestCameraSurfaceSize(width, height);
        resizeTextureView(width, height, cameraSize.getWidth(), cameraSize.getHeight());
        surfaceTexture.setDefaultBufferSize(cameraSize.getWidth(), cameraSize.getHeight());
        surface = new Surface(surfaceTexture);
        nativeCamera.startForSurface(surface);
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        nativeCamera.stop();
        surface = null;
        return true;
    }

    private void resizeTextureView(int textureWidth, int textureHeight,
                                   int cameraWidth, int cameraHeight) {
        double textureAspect = (double)textureWidth / textureHeight;
        double cameraAspect = (double)cameraWidth / cameraHeight;

        int newWidth = 0;
        int newHeight = 0;
        if (textureAspect >= cameraAspect) {
            newWidth = (int)(textureHeight * cameraAspect);
            newHeight = textureHeight;
        } else {
            newWidth = textureWidth;
            newHeight = (int)(textureWidth / cameraAspect);
        }

        Matrix transformMatrix = getTransformMatrix(
                getWindowManager().getDefaultDisplay().getRotation(), newWidth, newHeight);
        cameraView.setTransform(transformMatrix);
    }

    static Matrix getMatrixForPolyToPoly(float[] src, float[] dst) {
        Matrix matrix = new Matrix();
        matrix.setPolyToPoly(src, 0, dst, 0, 4);
        return matrix;
    }

    static Matrix getTransformMatrix(int rotation, int width, int height) {
        float[] defaultMatrixPoly = new float[]{
            0.f, 0.f,
            width, 0.f,
            0.f, height,
            width, height,
        };

        switch (rotation) {
            case 0: return new Matrix();
            case 1: // 90 degrees clockwise
                return getMatrixForPolyToPoly(defaultMatrixPoly, new float[]{
                    0.f, height,
                    0.f, 0.f,
                    width, height,
                    width, 0.f,
                });
            case 2: // 180 degrees clockwise
                return getMatrixForPolyToPoly(defaultMatrixPoly, new float[]{
                    width, height,
                    0.f, height,
                    width, 0.f,
                    0.f, 0.f,
                });
            case 3: // 270 degrees clockwise
                return getMatrixForPolyToPoly(defaultMatrixPoly, new float[]{
                    width, 0.f,
                    width, height,
                    0.f, 0.f,
                    0.f, height,
                });
            default: return new Matrix();
        }
    }

    @Override
    public void onResume()
    {
        super.onResume();

        if (ContextCompat.checkSelfPermission(
                getApplicationContext(), Manifest.permission.CAMERA) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.CAMERA},
                    PERMISSION_REQUEST_CAMERA);
            return;
        }

        setupCamera();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        if (requestCode != PERMISSION_REQUEST_CAMERA) {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
            return;
        }

        if (grantResults.length == 1 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            setupCamera();
        }
    }

    @Override
    public void onPause() {
        super.onPause();

        nativeCamera.close();
    }

    private void setupCamera() {
        nativeCamera.open();
        if (cameraView.isAvailable()) {
            onSurfaceTextureAvailable(cameraView.getSurfaceTexture(),
                    cameraView.getWidth(), cameraView.getHeight());
        }
    }
}