package com.example.crmaster;

import android.Manifest;
import androidx.annotation.NonNull;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;
import android.os.Handler;

public class MainActivity extends AppCompatActivity {

    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); // 通用UUID
    private static final int REQUEST_ENABLE_BT = 1;
    private static final int REQUEST_PERMISSION_BT = 2;
    private BluetoothAdapter bluetoothAdapter = null;
    private BluetoothSocket bluetoothSocket = null;
    private OutputStream outputStream = null;
    private TextView textViewLog;
private TextView textViewSeekBar1;
private TextView textViewSeekBar2;
    private byte[] data;
    private byte checksum;
    private int ang0=0;
    private int ang1=0;
    private int lastProgress = 0;
    private long lastUpdateTime = 0;
    private static final long MIN_UPDATE_INTERVAL = 50; // 最小更新间隔时间，单位为毫秒
    private Handler handler = new Handler();
    private Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            // 在这里调用你的sendData方法
            sendData();
        }
    };


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textViewLog = findViewById(R.id.textViewLog);
        SeekBar seekBar1 = findViewById(R.id.seekBar1);
        SeekBar seekBar2 = findViewById(R.id.seekBar2);
    textViewSeekBar1 = findViewById(R.id.textViewSeekBar1);
    textViewSeekBar2 = findViewById(R.id.textViewSeekBar2);
        seekBar1.setMax(180);
        seekBar2.setMax(180);

        seekBar1.setOnSeekBarChangeListener(seekBarChangeListener);
        seekBar2.setOnSeekBarChangeListener(seekBarChangeListener);

        // 初始化蓝牙适配器
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (bluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth is not available", Toast.LENGTH_SHORT).show();
            finish();
        }

        // 检查蓝牙权限
        checkBluetoothPermissions();
    }

    private void checkBluetoothPermissions() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH) != PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_ADMIN) != PackageManager.PERMISSION_GRANTED ||
            ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{
                            Manifest.permission.BLUETOOTH,
                            Manifest.permission.BLUETOOTH_ADMIN,
                            Manifest.permission.ACCESS_FINE_LOCATION
                    },
                    REQUEST_PERMISSION_BT);
        } else {
            checkBluetoothEnabled();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_PERMISSION_BT) {
            boolean allPermissionsGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allPermissionsGranted = false;
                    break;
                }
            }
            if (allPermissionsGranted) {
                checkBluetoothEnabled();
            } else {
                Toast.makeText(this, "Bluetooth permissions not granted", Toast.LENGTH_SHORT).show();
            }
        }
    }

    private void checkBluetoothEnabled() {
        if (!bluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        } else {
            connectToDevice("00:23:00:01:65:17"); // 替换为您的设备MAC地址
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == REQUEST_ENABLE_BT) {
            if (resultCode == RESULT_OK) {
                connectToDevice("00:23:00:01:65:17"); // 替换为您的设备MAC地址
            } else {
                Toast.makeText(this, "Bluetooth not enabled", Toast.LENGTH_SHORT).show();
            }
        }
    }
private SeekBar.OnSeekBarChangeListener seekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            if (fromUser && (System.currentTimeMillis() - lastUpdateTime > MIN_UPDATE_INTERVAL || progress == lastProgress)) {
                String hexData = String.format("0x%02X", progress);

                if (seekBar.getId() == R.id.seekBar1) {
                    ang0 = progress;
                    textViewSeekBar1.setText(String.valueOf(progress));
                    updateTextView("水平角: " + hexData);
                } else if (seekBar.getId() == R.id.seekBar2) {
                    ang1 = progress;
                    textViewSeekBar2.setText(String.valueOf(progress));
                    updateTextView("Z角: " + hexData);
                }

                int duty0 = (int) ((10.0 * ang0 / 180.0 + 2.5) * 20);
                int duty1 = (int) ((10.0 * ang1 / 180.0 + 2.5) * 20);

                 checksum = (byte) ((byte) 0xFF + (byte) ang0 + (byte) ang1 + (byte) (duty0 >> 8) + (byte) duty0 + (byte) (duty1 >> 8) + (byte) duty1);
                 data = new byte[]{(byte) 0xFF, (byte) ang0, (byte) ang1, (byte) (duty0 >> 8), (byte) duty0, (byte) (duty1 >> 8), (byte) duty1, (byte) checksum, (byte) 0xFE};

                lastProgress = progress;
                lastUpdateTime = System.currentTimeMillis();

                // 取消之前的延时任务
                handler.removeCallbacks(updateRunnable);
                // 重新安排延时任务
                handler.postDelayed(updateRunnable, MIN_UPDATE_INTERVAL);
            }
        }
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {
        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {
        }
    };

    private void connectToDevice(String deviceAddress) {
        BluetoothDevice device = bluetoothAdapter.getRemoteDevice(deviceAddress);
        try {
            bluetoothSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
            bluetoothSocket.connect();
            outputStream = bluetoothSocket.getOutputStream();
            updateTextView("Connected to device: " + device.getName());
        } catch (IOException e) {
            e.printStackTrace();
            updateTextView("Could not connect to device");
        }
    }

    private void sendData() {
        if (outputStream != null) {
            try {
                outputStream.write(data);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    private void updateTextView(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run(){
                textViewLog.setText(text);
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            if (bluetoothSocket != null) {
                bluetoothSocket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}