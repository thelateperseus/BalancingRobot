package com.example.barbiesegwayremote

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.*


class MainActivity : AppCompatActivity() {
    private var socket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var inputStream: InputStream? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
    }

    override fun onResume() {
        super.onResume()
        startBluetooth()
    }

    private fun toast(text: String) {
        Toast.makeText(this, text, Toast.LENGTH_SHORT).show()
    }

    private fun startBluetooth() {
        val bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
        if (bluetoothAdapter == null) {
            toast("No bluetooth adapter available")
            return
        }
        if (!bluetoothAdapter.isEnabled) {
            val enableBluetooth = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBluetooth, 0)
        }
        val pairedDevices: Set<BluetoothDevice> = bluetoothAdapter.bondedDevices
        val device = pairedDevices.find { d -> d.name == "BarbieSegway" }
        if (device == null) {
            toast("BarbieSegway NOT Found")
        } else {
            toast("BarbieSegway Found")

            // Standard SerialPortService ID
            val uuid: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
            val socket1 = device.createRfcommSocketToServiceRecord(uuid)
            try {
                socket1.connect()
                outputStream = socket1.getOutputStream()
                inputStream = socket1.getInputStream()
                socket = socket1
                toast("Bluetooth Opened")
            } catch (e: IOException) {
                toast("Exception: $e")
            }
        }
    }

    /** Called when the user taps the Forward button */
    fun forward(view: View) {
        outputStream?.write('F'.toInt());
    }
}
