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
import android.graphics.PixelFormat;
import android.os.Bundle;

import androidx.appcompat.app.AppCompatActivity;

import android.view.SurfaceHolder;
import android.view.SurfaceView;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.p12tic.mobisane.databinding.ActivityMainBinding;

import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;

public class MainActivity extends AppCompatActivity
        implements SurfaceHolder.Callback,
        ActivityCompat.OnRequestPermissionsResultCallback {
    static final int PERMISSION_REQUEST_CAMERA = 10;

    private NativeCamera nativeCamera = new NativeCamera();

    private ActivityMainBinding binding;

    private SurfaceView cameraView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        setSupportActionBar(binding.toolbar);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        cameraView = (SurfaceView) findViewById(R.id.cameraView);
        cameraView.getHolder().addCallback(this);
        cameraView.getHolder().setFormat(PixelFormat.RGBA_8888);
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
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height)
    {
        nativeCamera.setSurface(holder.getSurface());
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder)
    {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder)
    {
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

        nativeCamera.open();
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
            nativeCamera.open();
        }
    }

    @Override
    public void onPause()
    {
        super.onPause();

        nativeCamera.close();
    }

    static {
        System.loadLibrary("mobisane");
    }
}