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

import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class ResourceDownloadAsyncTask extends AsyncTask<String, Integer, Integer> {

    private Context context;
    private Consumer<Integer> onComplete;
    private Consumer<Double> onProgress;

    public static final int SUCCESS = 0;
    public static final int CANCELLED = 1;
    public static final int FAILURE = 2;
    public static final int FAILURE_NO_NETWORK = 3;

    public ResourceDownloadAsyncTask(Context context, Consumer<Double> onProgress,
                                     Consumer<Integer> onComplete) {
        this.context = context;
        this.onProgress = onProgress;
        this.onComplete = onComplete;
    }

    private String getInternalFilesDir() {
        return context.getFilesDir().getAbsolutePath();
    }

    private int downloadFile(String dstPath, String urlString,
                             int currTotalBytesAllDownloads, int expectedTotalBytesAllDownloads) {

        InputStream input = null;
        OutputStream output = null;
        HttpURLConnection connection = null;

        try {
            File dstFile = new File(getInternalFilesDir() + "/" + dstPath);
            Log.println(Log.DEBUG, "mobisane", "Started downloading " + dstPath);

            if (dstFile.exists()) {
                Log.println(Log.DEBUG, "mobisane", "Already exists " + dstPath);
                return SUCCESS;
            }

            URL url = new URL(urlString);
            connection = (HttpURLConnection) url.openConnection();
            connection.connect();

            if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
                Log.println(Log.DEBUG, "mobisane", "Got HTTP failure: " + connection.getResponseCode());
                return FAILURE_NO_NETWORK;
            }

            input = connection.getInputStream();
            output = new FileOutputStream(dstFile.getPath());

            byte[] data = new byte[40960];
            int count;
            while ((count = input.read(data)) != -1) {
                if (isCancelled()) {
                    input.close();
                    dstFile.delete();
                    return CANCELLED;
                }
                output.write(data, 0, count);
                currTotalBytesAllDownloads += count;
                onProgress.accept((double)currTotalBytesAllDownloads / expectedTotalBytesAllDownloads);
            }
        } catch (Exception e) {
            Log.println(Log.ERROR, "mobisane", "Got exception: " + e);
            return FAILURE;
        } finally {
            try {
                if (output != null) {
                    output.close();
                }
                if (input != null) {
                    input.close();
                }
            } catch (IOException e) {
            }

            if (connection != null) {
                connection.disconnect();
            }
        }
        return SUCCESS;
    }

    protected class DownloadConfig {
        public String filename;
        public String url;
        public int size;
        public DownloadConfig(String filename, String url, int size) {
            this.filename = filename;
            this.url = url;
            this.size = size;
        }
    }
    @Override
    protected Integer doInBackground(String... unused) {

        InputStream input = null;
        OutputStream output = null;
        HttpURLConnection connection = null;

        List<DownloadConfig> downloadConfigs = new ArrayList<DownloadConfig>();
        downloadConfigs.add(new DownloadConfig(
                "cameraSensors.db",
                "https://raw.githubusercontent.com/alicevision/AliceVision/develop/src/aliceVision/sensorDB/cameraSensors.db",
                359897
        ));
        downloadConfigs.add(new DownloadConfig(
                "eng.traineddata",
                "https://github.com/tesseract-ocr/tessdata/raw/main/eng.traineddata",
                23466654
        ));
        downloadConfigs.add(new DownloadConfig(
                "osd.traineddata",
                "https://github.com/tesseract-ocr/tessdata/raw/main/osd.traineddata",
                10562874
        ));

        int expectedTotalBytesAllDownloads = 0;
        for (DownloadConfig config : downloadConfigs) {
            expectedTotalBytesAllDownloads += config.size;
        }

        try {
            // HACK: getContentLength() doesn't work properly, so we hardcode the number of download bytes for now
            int currTotalBytesAllDownloads = 0;
            for (DownloadConfig config : downloadConfigs) {
                int result;
                result = downloadFile(config.filename, config.url, currTotalBytesAllDownloads,
                        expectedTotalBytesAllDownloads);

                if (result != SUCCESS) {
                    return result;
                }
                currTotalBytesAllDownloads += config.size;
            }
            return SUCCESS;
        } catch (Exception e) {
            Log.println(Log.ERROR, "mobisane", "Got exception: " + e);
        }
        return FAILURE;
    }

    @Override
    protected void onPostExecute(Integer s) {
        super.onPostExecute(s);
        onComplete.accept(s);
    }
}