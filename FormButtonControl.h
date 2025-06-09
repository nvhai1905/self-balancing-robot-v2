#pragma once

namespace bbb {

    using namespace System;
    using namespace System::ComponentModel;
    using namespace System::Collections;
    using namespace System::Windows::Forms;
    using namespace System::Data;
    using namespace System::Drawing;
    using namespace System::IO::Ports;

    public ref class FormButtonControl : public System::Windows::Forms::Form
    {
    private:
        SerialPort^ serialPort;
        String^ receivedData; // Lưu dữ liệu nhận từ Serial

    public:
        FormButtonControl(void)
        {
            InitializeComponent();
            serialPort = nullptr;
            receivedData = "";
            LoadCOMPorts();
        }

    protected:
        ~FormButtonControl()
        {
            if (components)
            {
                delete components;
            }
            if (serialPort != nullptr && serialPort->IsOpen)
            {
                serialPort->Close();
                delete serialPort;
            }
        }

    private: System::Windows::Forms::Label^ label1;
    private: System::Windows::Forms::Button^ button1;
    private: System::Windows::Forms::Button^ button2;
    private: System::Windows::Forms::Button^ button3;
    private: System::Windows::Forms::Button^ button4;
    private: System::Windows::Forms::Label^ labelStatus;
    private: System::Windows::Forms::PictureBox^ pictureBox1;
    private: System::Windows::Forms::PictureBox^ pictureBox2;
    private: System::Windows::Forms::Label^ label2;
    private: System::Windows::Forms::ComboBox^ comboBoxCOM;
    private: System::Windows::Forms::Button^ btnConnect;

    private:
        System::ComponentModel::Container^ components;

#pragma region Windows Form Designer generated code
        void InitializeComponent(void)
        {
            System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(FormButtonControl::typeid));
            this->label1 = (gcnew System::Windows::Forms::Label());
            this->button1 = (gcnew System::Windows::Forms::Button());
            this->button2 = (gcnew System::Windows::Forms::Button());
            this->button3 = (gcnew System::Windows::Forms::Button());
            this->button4 = (gcnew System::Windows::Forms::Button());
            this->labelStatus = (gcnew System::Windows::Forms::Label());
            this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
            this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
            this->label2 = (gcnew System::Windows::Forms::Label());
            this->comboBoxCOM = (gcnew System::Windows::Forms::ComboBox());
            this->btnConnect = (gcnew System::Windows::Forms::Button());
            (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
            (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
            this->SuspendLayout();
            // 
            // label1
            // 
            this->label1->AutoSize = true;
            this->label1->BackColor = System::Drawing::SystemColors::ActiveCaption;
            this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->label1->Location = System::Drawing::Point(376, 51);
            this->label1->Name = L"label1";
            this->label1->Size = System::Drawing::Size(254, 29);
            this->label1->TabIndex = 0;
            this->label1->Text = L"BUTTON CONTROL";
            // 
            // button1
            // 
            this->button1->BackColor = System::Drawing::Color::Cornsilk;
            this->button1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->button1->Location = System::Drawing::Point(100, 270);
            this->button1->Name = L"button1";
            this->button1->Size = System::Drawing::Size(208, 69);
            this->button1->TabIndex = 1;
            this->button1->Text = L"Turning left";
            this->button1->UseVisualStyleBackColor = false;
            this->button1->Click += gcnew System::EventHandler(this, &FormButtonControl::button1_Click);
            // 
            // button2
            // 
            this->button2->BackColor = System::Drawing::SystemColors::Highlight;
            this->button2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->button2->Location = System::Drawing::Point(685, 270);
            this->button2->Name = L"button2";
            this->button2->Size = System::Drawing::Size(212, 69);
            this->button2->TabIndex = 2;
            this->button2->Text = L"Turning right";
            this->button2->UseVisualStyleBackColor = false;
            this->button2->Click += gcnew System::EventHandler(this, &FormButtonControl::button2_Click);
            // 
            // button3
            // 
            this->button3->BackColor = System::Drawing::Color::IndianRed;
            this->button3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->button3->Location = System::Drawing::Point(381, 159);
            this->button3->Name = L"button3";
            this->button3->Size = System::Drawing::Size(249, 59);
            this->button3->TabIndex = 3;
            this->button3->Text = L"Moving forward";
            this->button3->UseVisualStyleBackColor = false;
            this->button3->Click += gcnew System::EventHandler(this, &FormButtonControl::button3_Click);
            // 
            // button4
            // 
            this->button4->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(128)), static_cast<System::Int32>(static_cast<System::Byte>(64)),
                static_cast<System::Int32>(static_cast<System::Byte>(0)));
            this->button4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->button4->Location = System::Drawing::Point(381, 462);
            this->button4->Name = L"button4";
            this->button4->Size = System::Drawing::Size(249, 60);
            this->button4->TabIndex = 4;
            this->button4->Text = L"Moving backward";
            this->button4->UseVisualStyleBackColor = false;
            this->button4->Click += gcnew System::EventHandler(this, &FormButtonControl::button4_Click);
            // 
            // labelStatus
            // 
            this->labelStatus->AutoSize = true;
            this->labelStatus->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 13.8F, System::Drawing::FontStyle::Italic, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->labelStatus->Location = System::Drawing::Point(423, 300);
            this->labelStatus->Name = L"labelStatus";
            this->labelStatus->Size = System::Drawing::Size(109, 29);
            this->labelStatus->TabIndex = 5;
            this->labelStatus->Text = L"STATUS";
            // 
            // pictureBox1
            // 
            this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox1.Image")));
            this->pictureBox1->Location = System::Drawing::Point(910, 549);
            this->pictureBox1->Name = L"pictureBox1";
            this->pictureBox1->Size = System::Drawing::Size(100, 116);
            this->pictureBox1->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
            this->pictureBox1->TabIndex = 7;
            this->pictureBox1->TabStop = false;
            // 
            // pictureBox2
            // 
            this->pictureBox2->Image = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"pictureBox2.Image")));
            this->pictureBox2->Location = System::Drawing::Point(12, 549);
            this->pictureBox2->Name = L"pictureBox2";
            this->pictureBox2->Size = System::Drawing::Size(115, 116);
            this->pictureBox2->SizeMode = System::Windows::Forms::PictureBoxSizeMode::Zoom;
            this->pictureBox2->TabIndex = 8;
            this->pictureBox2->TabStop = false;
            // 
            // label2
            // 
            this->label2->AutoSize = true;
            this->label2->BackColor = System::Drawing::Color::RosyBrown;
            this->label2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
                static_cast<System::Byte>(163)));
            this->label2->Location = System::Drawing::Point(69, 61);
            this->label2->Name = L"label2";
            this->label2->Size = System::Drawing::Size(90, 25);
            this->label2->TabIndex = 10;
            this->label2->Text = L"Angle: 0 ";
            // 
            // comboBoxCOM
            // 
            this->comboBoxCOM->FormattingEnabled = true;
            this->comboBoxCOM->Location = System::Drawing::Point(816, 134);
            this->comboBoxCOM->Name = L"comboBoxCOM";
            this->comboBoxCOM->Size = System::Drawing::Size(121, 24);
            this->comboBoxCOM->TabIndex = 11;
            // 
            // btnConnect
            // 
            this->btnConnect->BackColor = System::Drawing::Color::Firebrick;
            this->btnConnect->Location = System::Drawing::Point(816, 105);
            this->btnConnect->Name = L"btnConnect";
            this->btnConnect->Size = System::Drawing::Size(75, 23);
            this->btnConnect->TabIndex = 12;
            this->btnConnect->Text = L"Connect";
            this->btnConnect->UseVisualStyleBackColor = false;
            this->btnConnect->Click += gcnew System::EventHandler(this, &FormButtonControl::btnConnect_Click);
            // 
            // FormButtonControl
            // 
            this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
            this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
            this->ClientSize = System::Drawing::Size(1022, 677);
            this->Controls->Add(this->btnConnect);
            this->Controls->Add(this->comboBoxCOM);
            this->Controls->Add(this->label2);
            this->Controls->Add(this->pictureBox2);
            this->Controls->Add(this->pictureBox1);
            this->Controls->Add(this->labelStatus);
            this->Controls->Add(this->button4);
            this->Controls->Add(this->button3);
            this->Controls->Add(this->button2);
            this->Controls->Add(this->button1);
            this->Controls->Add(this->label1);
            this->Name = L"FormButtonControl";
            this->Text = L"FormButtonControl";
            (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
            (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
            this->ResumeLayout(false);
            this->PerformLayout();
        }
#pragma endregion

    private:
        void LoadCOMPorts()
        {
            array<String^>^ ports = SerialPort::GetPortNames();
            comboBoxCOM->Items->Clear();
            for each(String ^ port in ports)
            {
                comboBoxCOM->Items->Add(port);
                Console::WriteLine("Found port: " + port);
            }
            if (comboBoxCOM->Items->Count > 0)
                comboBoxCOM->SelectedIndex = 0;
        }

        void SendCommand(String^ command, String^ displayText)
        {
            if (serialPort != nullptr && serialPort->IsOpen)
            {
                serialPort->WriteLine(command);
                labelStatus->Text = displayText + "...";
            }
            else
            {
                MessageBox::Show("Please connect to a COM port first!", "Connection Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
            }
        }

        // Hàm xử lý khi nhận dữ liệu từ Serial
        void SerialPort_DataReceived(Object^ sender, SerialDataReceivedEventArgs^ e)
        {
            try
            {
                String^ data = serialPort->ReadLine();
                receivedData += data + "\n";

                // Tìm và xử lý dữ liệu góc
                array<String^>^ lines = receivedData->Split(gcnew array<String^>{"\n"}, StringSplitOptions::RemoveEmptyEntries);
                for each(String ^ line in lines)
                {
                    if (line->StartsWith("ANGLE:"))
                    {
                        String^ angleStr = line->Substring(6); // Bỏ phần "ANGLE:"
                        double angle;
                        if (Double::TryParse(angleStr, angle))
                        {
                            // Cập nhật label2 trên luồng giao diện
                            label2->Invoke(gcnew Action<double>(this, &FormButtonControl::UpdateAngleLabel), angle);
                        }
                    }
                }
                receivedData = ""; // Xóa dữ liệu đã xử lý
            }
            catch (Exception^ ex)
            {
                MessageBox::Show("Error reading from Serial: " + ex->Message, "Serial Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
            }
        }

        // Hàm cập nhật label2 với giá trị góc
        void UpdateAngleLabel(double angle)
        {
            label2->Text = "Angle: " + angle.ToString("F2"); // Hiển thị với 2 chữ số thập phân
        }

    private: System::Void button3_Click(System::Object^ sender, System::EventArgs^ e) {
        SendCommand("F", "Moving forward");
    }

    private: System::Void button4_Click(System::Object^ sender, System::EventArgs^ e) {
        SendCommand("B", "Moving backward");
    }

    private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
        SendCommand("L", "Turning left");
    }

    private: System::Void button2_Click(System::Object^ sender, System::EventArgs^ e) {
        SendCommand("R", "Turning right");
    }

    private: System::Void btnConnect_Click(System::Object^ sender, System::EventArgs^ e) {
        try
        {
            if (serialPort != nullptr && serialPort->IsOpen)
            {
                serialPort->Close();
            }

            if (comboBoxCOM->SelectedItem == nullptr)
            {
                MessageBox::Show("Please select a COM port!", "Selection Error", MessageBoxButtons::OK, MessageBoxIcon::Warning);
                return;
            }

            String^ selectedPort = comboBoxCOM->SelectedItem->ToString();
            serialPort = gcnew SerialPort(selectedPort, 115200);
            serialPort->ReadTimeout = 500;
            serialPort->WriteTimeout = 500;
            serialPort->DataReceived += gcnew SerialDataReceivedEventHandler(this, &FormButtonControl::SerialPort_DataReceived);
            serialPort->Open();
            labelStatus->Text = "Connected to " + selectedPort;
            MessageBox::Show("Connected successfully to " + selectedPort, "Success", MessageBoxButtons::OK, MessageBoxIcon::Information);
        }
        catch (Exception^ ex)
        {
            MessageBox::Show("Connection error: " + ex->Message + "\nPort: " + (comboBoxCOM->SelectedItem != nullptr ? comboBoxCOM->SelectedItem->ToString() : "None"), "Error", MessageBoxButtons::OK, MessageBoxIcon::Error);
        }
    }
    };
}