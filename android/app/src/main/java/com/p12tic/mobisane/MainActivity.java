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
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.net.Uri;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import android.util.Size;
import android.view.Menu;
import android.view.Surface;

import androidx.core.app.ActivityCompat;
import androidx.core.app.ShareCompat;
import androidx.core.content.ContextCompat;
import androidx.core.content.FileProvider;

import com.google.android.material.floatingactionbutton.FloatingActionButton;
import com.p12tic.mobisane.databinding.ActivityMainBinding;

import android.view.MenuItem;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.ProgressBar;
import android.widget.TextView;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;

public class MainActivity extends AppCompatActivity
        implements ActivityCompat.OnRequestPermissionsResultCallback {
    static final int PERMISSION_REQUEST_CAMERA = 10;
    static final int MIN_NUMBER_OF_IMAGES = 3;

    private NativeCamera nativeCamera = new NativeCamera();
    private NativeAppManager nativeAppManager = new NativeAppManager();

    Handler handler = new Handler();
    Runnable progressRefreshRunnable;
    static final int PROGRESS_REFRESH_INTERVAL_MS = 250;

    private ActivityMainBinding binding;

    private boolean initialized = false;
    private boolean gotCameraPermission = false;
    private boolean isRunningAnalysis = false;
    private String destinationScanPath;

    private TextureView cameraView;
    private TextureView overlayView;
    private Surface cameraViewSurface = null;
    private Surface overlayViewSurface = null;
    private TextView infoTextLabel;
    private ProgressBar progressBar;

    private Menu menu;
    private MenuItem analyzeImages;

    private int numberOfPendingImages = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        setSupportActionBar(binding.toolbar);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        infoTextLabel = (TextView) findViewById(R.id.textView);
        infoTextLabel.setText("Loading resources...");
        progressBar = (ProgressBar) findViewById(R.id.progressBar);
        progressBar.setAlpha(0);

        new ResourceDownloadAsyncTask(getApplicationContext(), (result) -> {
            if (result == ResourceDownloadAsyncTask.SUCCESS)  {
                infoTextLabel.setText("");
                nativeAppManager.notifyResourcesReady(
                        getApplicationContext().getFilesDir().getAbsolutePath());
                setupTextureView();
                initialized = true;
                maybeSetupCamera();
                prepareForNewSession();
            } else {
                infoTextLabel.setText("Resource loading failure... Please enable network");
            }
        }).execute();
    }

    private void setupTextureView() {
        cameraView = (TextureView) findViewById(R.id.cameraView);
        cameraView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture,
                                                  int width, int height) {
                onPreviewSurfaceTextureAvailable(surfaceTexture, width, height);
            }

            @Override
            public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surfaceTexture,
                                                    int width, int height) { }

            @Override
            public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surfaceTexture) {
                return onPreviewSurfaceTextureDestroyed();
            }

            @Override
            public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surfaceTexture) { }
        });

        overlayView = (TextureView) findViewById(R.id.overlayView);
        overlayView.setOpaque(false);
        overlayView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture,
                                                  int width, int height) {
                onOverlaySurfaceTextureAvailable(surfaceTexture, width, height);
            }

            @Override
            public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surfaceTexture,
                                                    int width, int height) { }

            @Override
            public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surfaceTexture) {
                return onOverlaySurfaceTextureDestroyed();
            }

            @Override
            public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surfaceTexture) { }
        });

        Activity self = this;
        FloatingActionButton takePhotoButton = (FloatingActionButton) findViewById(R.id.takePhoto);
        takePhotoButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (nativeCamera.isOpen()) {
                    nativeCamera.captureImage();
                    numberOfPendingImages++;
                    updateMenuItemStatuses();
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
    public boolean onPrepareOptionsMenu(Menu menu) {
        boolean res = super.onPrepareOptionsMenu(menu);
        this.menu = menu;
        analyzeImages = menu.findItem(R.id.action_analyze);
        updateMenuItemStatuses();
        return res;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_analyze) {
            numberOfPendingImages = 0;
            isRunningAnalysis = true; // the status will be handled by the status refreshing code
            nativeAppManager.startAnalysis();
            updateMenuItemStatuses();
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    private void updateMenuItemStatuses() {
        if (analyzeImages != null) {
            analyzeImages.setEnabled(numberOfPendingImages >= MIN_NUMBER_OF_IMAGES);
        }
    }

    void rotateTexture(TextureView view, int rotation, int srcWidth, int srcHeight,
                       int dstWidth, int dstHeight) {
        if (rotation == 0) {
            return;
        }

        Matrix transformMatrix = getTransformMatrix(rotation, dstWidth, dstHeight);
        view.setTransform(transformMatrix);
    }

    void onPreviewSurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture,
                                          int width, int height) {
        if (!nativeCamera.isOpen()) {
            return;
        }

        Size cameraSize = nativeCamera.getBestCameraSurfaceSize(width, height);
        surfaceTexture.setDefaultBufferSize(cameraSize.getWidth(), cameraSize.getHeight());
        cameraViewSurface = new Surface(surfaceTexture);
        nativeCamera.startForSurface(cameraViewSurface);
    }

    boolean onPreviewSurfaceTextureDestroyed() {
        nativeCamera.stop();
        cameraViewSurface = null;
        return true;
    }

    void onOverlaySurfaceTextureAvailable(@NonNull SurfaceTexture surfaceTexture,
                                          int width, int height) {
        if (!nativeCamera.isOpen()) {
            return;
        }

        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        rotation += nativeCamera.getBestCameraRotation();
        rotation %= 4;

        Size cameraSize = nativeCamera.getBestCameraSurfaceSize(width, height);
        surfaceTexture.setDefaultBufferSize(cameraSize.getWidth(), cameraSize.getHeight());
        rotateTexture(overlayView, rotation, cameraSize.getWidth(), cameraSize.getHeight(),
                      width, height);
        overlayViewSurface = new Surface(surfaceTexture);
        nativeAppManager.setPreviewSurface(overlayViewSurface);
    }

    boolean onOverlaySurfaceTextureDestroyed() {
        overlayViewSurface = null;
        nativeAppManager.setPreviewSurface(null);
        return true;
    }

    static Matrix getMatrixForPolyToPoly(float[] src, float[] dst) {
        Matrix matrix = new Matrix();
        if (!matrix.setPolyToPoly(src, 0, dst, 0, 4)) {
            Log.println(Log.ERROR, "failure", "failure");
        }
        return matrix;
    }

    static Matrix getTransformMatrix(int rotation, int width, int height) {
        float[] srcMatrixPoly = new float[]{
            0.f, 0.f,
            width, 0.f,
            0.f, height,
            width, height,
        };

        switch (rotation) {
            case 0:
                return getMatrixForPolyToPoly(srcMatrixPoly, new float[]{
                    0.f, 0.f,
                    width, 0.f,
                    0.f, height,
                    width, height,
                });
            case 1: // 90 degrees clockwise
                return getMatrixForPolyToPoly(srcMatrixPoly, new float[]{
                        width, 0.f,
                        width, height,
                        0.f, 0.f,
                        0.f, height,
                });
            case 2: // 180 degrees clockwise
                return getMatrixForPolyToPoly(srcMatrixPoly, new float[]{
                    width, height,
                    0.f, height,
                    width, 0.f,
                    0.f, 0.f,
                });
            case 3: // 270 degrees clockwise
                return getMatrixForPolyToPoly(srcMatrixPoly, new float[]{
                        0.f, height,
                        0.f, 0.f,
                        width, height,
                        width, 0.f,
                });
            default: return new Matrix();
        }
    }

    @Override
    public void onResume()
    {
        gotCameraPermission = false;
        super.onResume();

        enableProgressReporting();

        if (ContextCompat.checkSelfPermission(
                getApplicationContext(), Manifest.permission.CAMERA) == PackageManager.PERMISSION_DENIED)
        {
            ActivityCompat.requestPermissions(this, new String[] {Manifest.permission.CAMERA},
                    PERMISSION_REQUEST_CAMERA);
            return;
        }

        gotCameraPermission = true;
        maybeSetupCamera();
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
            gotCameraPermission = true;
            maybeSetupCamera();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        disableProgressReporting();

        nativeCamera.close();
    }

    private void maybeSetupCamera() {
        if (!gotCameraPermission) {
            return;
        }
        if (!initialized) {
            return;
        }

        nativeCamera.open();
        if (cameraView.isAvailable()) {
            onPreviewSurfaceTextureAvailable(cameraView.getSurfaceTexture(),
                    cameraView.getWidth(), cameraView.getHeight());
        }
        if (overlayView.isAvailable()) {
            onOverlaySurfaceTextureAvailable(overlayView.getSurfaceTexture(),
                    overlayView.getWidth(), overlayView.getHeight());
        }
    }


    private void enableProgressReporting() {
        handler.postDelayed(progressRefreshRunnable = new Runnable() {
            public void run() {
                handler.postDelayed(progressRefreshRunnable, PROGRESS_REFRESH_INTERVAL_MS);

                if (!initialized) {
                    return;
                }
                double progress = nativeAppManager.getCurrentProgress();
                if (Double.isNaN(progress)) {
                    progressBar.setAlpha(0);
                } else {
                    progressBar.setAlpha(1);
                    progressBar.setMax(100);
                    progressBar.setProgress((int)(progress * 100));
                }
                String status = nativeAppManager.getCurrentStatus();
                if (status.isEmpty()) {
                    infoTextLabel.setText("");
                } else {
                    infoTextLabel.setText(status);
                }

                if (isRunningAnalysis && nativeAppManager.isAnalysisFinished()) {
                    isRunningAnalysis = false;
                    if (nativeAppManager.isAnalysisSuccess()) {
                        String scanPath = destinationScanPath;
                        prepareForNewSession();
                        invokePdfViewer(scanPath);
                    }
                }
            }
        }, PROGRESS_REFRESH_INTERVAL_MS);
    }

    private void disableProgressReporting() {
        handler.removeCallbacks(progressRefreshRunnable);
    }

    private void prepareForNewSession() {
        File destDirectory = Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS);
        String filename = new SimpleDateFormat("'scanned-'yyyy-MM-dd_hh-mm-ss'.pdf'").format(new Date());
        destinationScanPath = destDirectory + "/" + filename;
        nativeAppManager.startNewSession(destinationScanPath);
    }

    private void invokePdfViewer(String pdfPath) {
        File file = new File(pdfPath);
        Uri uri = FileProvider.getUriForFile(this, getPackageName(), file);

        Intent intent = ShareCompat.IntentBuilder.from(this)
                .setStream(uri) // uri from FileProvider
                .setType("application/pdf")
                .getIntent()
                .setAction(Intent.ACTION_VIEW) //Change if needed
                .setDataAndType(uri, "application/pdf")
                .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);

        startActivity(Intent.createChooser(intent, "Select application to open scanned file"));
    }
}