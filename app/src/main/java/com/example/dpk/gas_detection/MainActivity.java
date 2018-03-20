package com.example.dpk.gas_detection;

import android.content.Intent;
import android.os.Build;
import android.support.annotation.RequiresApi;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Objects;

import static android.content.ContentValues.TAG;

public class MainActivity extends AppCompatActivity {

    private Socket client;
    private FileInputStream fileInputStream;
    private BufferedInputStream bufferedInputStream;
    private OutputStream outputStream;
    ImageView i1,i2,i3,i4,a1,a2,a3,a4;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        a1=findViewById(R.id.imageView2a);
        a2=findViewById(R.id.imageView1a);
        a3=findViewById(R.id.imageView2b);
        a4=findViewById(R.id.imageView1b);
        i1=findViewById(R.id.imageView2c);
        i2=findViewById(R.id.imageView1c);
        i3=findViewById(R.id.imageView2d);
        i4=findViewById(R.id.imageView1d);




      Thread thread = new Thread(new Runnable() {

            @RequiresApi(api = Build.VERSION_CODES.KITKAT)
            @Override
            public void run() {

                try {



                    client = new Socket("192.168.43.47", 12345);// ip address and port number of our localhost

                    InputStream is =client.getInputStream();
                    BufferedReader r = new BufferedReader(new InputStreamReader(is));
                    String data =(String) r.readLine();
                    Log.d(TAG, "run: "+data);

                    if(Objects.equals(data, "1")){
                        Log.d(TAG, "run: "+data);
                        a2.setVisibility(View.GONE);
                        a1.setVisibility(View.VISIBLE);
                    }

                    if(Objects.equals(data, "2")){
                        Log.d(TAG, "run: "+data);
                        a4.setVisibility(View.GONE);
                        a3.setVisibility(View.VISIBLE);
                    }

                    if(Objects.equals(data, "3")){
                        Log.d(TAG, "run: "+data);
                        i4.setVisibility(View.GONE);
                        i3.setVisibility(View.VISIBLE);
                    }

                    if(Objects.equals(data, "4")){
                        Log.d(TAG, "run: "+data);
                        i2.setVisibility(View.GONE);
                        i1.setVisibility(View.VISIBLE);
                    }



                    outputStream = client.getOutputStream();
                    outputStream.flush();
                    bufferedInputStream.close();
                    outputStream.close();
                    client.close();


                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });

        thread.start();

    }
}
