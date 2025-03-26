#include <iostream>
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include <string>
#include <XInput.h>
#include <cmath>
#include <locale>
#include <cstdint>
#include <cstdlib>
#include <conio.h>

using namespace std;

#pragma comment(lib, "XInput.lib")

// Структура для установки локали
struct SetLocale {
    SetLocale() {
        try {
            locale::global(locale(""));
        } catch (const runtime_error&) {
            locale::global(locale("C"));
        }
    }
} setLocale;

// Функция вывода ошибок
void PrintError(const wstring& message) {
    DWORD error = GetLastError();
    LPWSTR lpMsgBuf = nullptr;

    FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPWSTR)&lpMsgBuf,
        0,
        NULL
    );

    wcerr << message << L": " << (lpMsgBuf ? lpMsgBuf : L"Unknown error") << endl;

    if (lpMsgBuf) LocalFree(lpMsgBuf);
}

// Функция расчета CRC16
uint16_t crc16_ccitt(const uint8_t* data, int length) {
    uint16_t crc = 0xFFFF;
    
    for (int i = 1; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    
    return crc;
}

// Состояние геймпада
struct GamepadState {
    bool A, B, X, Y;
    bool LB, RB;
    bool LeftThumb, RightThumb;  
    bool Start, Back;
    bool DPad_Up, DPad_Down, DPad_Left, DPad_Right;

    int16_t LeftThumbX, LeftThumbY;
    int16_t RightThumbX, RightThumbY;
    uint8_t LeftTrigger, RightTrigger;
};

// Нормализация значений стиков
int16_t normalizeStickValue(SHORT rawValue, SHORT deadZone = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) {
    if (abs(rawValue) < deadZone) return 0;
    return rawValue;
}

// Получение состояния геймпада
bool GetGamepadState(int controllerIndex, GamepadState& state) {
    XINPUT_STATE xState;
    ZeroMemory(&xState, sizeof(XINPUT_STATE));

    if (XInputGetState(controllerIndex, &xState) != ERROR_SUCCESS)
        return false;

    // Кнопки
    state.A = xState.Gamepad.wButtons & XINPUT_GAMEPAD_A;
    state.B = xState.Gamepad.wButtons & XINPUT_GAMEPAD_B;
    state.X = xState.Gamepad.wButtons & XINPUT_GAMEPAD_X;
    state.Y = xState.Gamepad.wButtons & XINPUT_GAMEPAD_Y;
    state.LB = xState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER;
    state.RB = xState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER;
    state.LeftThumb = xState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB;
    state.RightThumb = xState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB;
    state.Start = xState.Gamepad.wButtons & XINPUT_GAMEPAD_START;
    state.Back = xState.Gamepad.wButtons & XINPUT_GAMEPAD_BACK;

    // D-Pad
    state.DPad_Up = xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP;
    state.DPad_Down = xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN;
    state.DPad_Left = xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT;
    state.DPad_Right = xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT;

    // Стики
    state.LeftThumbX = normalizeStickValue(xState.Gamepad.sThumbLX);
    state.LeftThumbY = normalizeStickValue(xState.Gamepad.sThumbLY);
    state.RightThumbX = normalizeStickValue(xState.Gamepad.sThumbRX);
    state.RightThumbY = normalizeStickValue(xState.Gamepad.sThumbRY);

    // Триггеры
    state.LeftTrigger = xState.Gamepad.bLeftTrigger;
    state.RightTrigger = xState.Gamepad.bRightTrigger;

    return true;
}

int main() {
    // Настройка консоли
    _setmode(_fileno(stdout), _O_U8TEXT);
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);

    // Инициализация XInput
    HMODULE hXInput = LoadLibraryW(L"XInput1_4.dll");
    if (!hXInput) hXInput = LoadLibraryW(L"XInput9_1_0.dll");
    if (!hXInput) {
        PrintError(L"Ошибка загрузки XInput");
        return 1;
    }

    // Открытие COM-порта
    HANDLE hSerial = CreateFileW(
        L"\\\\.\\COM1",
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        PrintError(L"Ошибка открытия COM-порта");
        FreeLibrary(hXInput);
        return 1;
    }

    // Настройка параметров порта
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hSerial, &dcb)) {
        PrintError(L"Ошибка получения параметров порта");
        CloseHandle(hSerial);
        FreeLibrary(hXInput);
        return 1;
    }

    dcb.BaudRate = CBR_19200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(hSerial, &dcb)) {
        PrintError(L"Ошибка настройки параметров порта");
        CloseHandle(hSerial);
        FreeLibrary(hXInput);
        return 1;
    }

    // Подготовка структуры для асинхронной записи
    OVERLAPPED ov = {0};
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!ov.hEvent) {
        PrintError(L"Ошибка создания события");
        CloseHandle(hSerial);
        FreeLibrary(hXInput);
        return 1;
    }

    wcout << L"Программа запущена. Нажмите ESC для выхода...\n";

    // Главный цикл
    while (true) {
        if (_kbhit() && _getch() == 27) break; // Выход по ESC

        GamepadState state;
        if (!GetGamepadState(0, state)) {
            wcout << L"Геймпад не подключен!\n";
            Sleep(1000);
            continue;
        }

        // Формирование пакета данных
        uint8_t buffer[13] = {0};

        buffer[0] = 0x0A; // Заголовок пакета

        // Байт 1: основные кнопки
        buffer[1] |= state.A << 7;          //кнопка А
        buffer[1] |= state.B << 6;          //кнопка В
        buffer[1] |= state.X << 5;          //кнопка Х
        buffer[1] |= state.Y << 4;          //кнопка Y
        buffer[1] |= state.DPad_Up << 3;    // DPad вверх
        buffer[1] |= state.DPad_Down << 2;  // DPad вниз
        buffer[1] |= state.DPad_Left << 1;  // D Pad влево
        buffer[1] |= state.DPad_Right;      // D Pad вправо

        // Стики
        buffer[2] = (state.LeftThumbX >> 8) & 0xFF;     //  левый стик Х
        buffer[3] = state.LeftThumbX & 0xFF;            //  левый стик Х (младшие)
        buffer[4] = (state.LeftThumbY >> 8) & 0xFF;     //  левый стик Y
        buffer[5] = state.LeftThumbY & 0xFF;            //  левый стик Y (младшие)
                                                    
        buffer[6] = (state.RightThumbX >> 8) & 0xFF;    //  правый стик Х
        buffer[7] = state.RightThumbX & 0xFF;           //  правый стик Х (младшие)
        buffer[8] = (state.RightThumbY >> 8) & 0xFF;    //  правый стик Y
        buffer[9] = state.RightThumbY & 0xFF;           //  правый стик Y (младшие)

        // Байт 10: дополнительные кнопки
        buffer[10] |= state.LeftThumb << 7;
        buffer[10] |= state.RightThumb << 6;
        buffer[10] |= state.LB << 5;
        buffer[10] |= state.RB << 4;
        buffer[10] |= (state.LeftTrigger > 0) << 3;
        buffer[10] |= (state.RightTrigger > 0) << 2;
        buffer[10] |= state.Start << 1;
        buffer[10] |= state.Back;

        // Расчет CRC
        uint16_t crc = crc16_ccitt(buffer, 11);
        buffer[11] = (crc >> 8) & 0xFF;
        buffer[12] = crc & 0xFF;

        // Отправка данных
        DWORD bytesWritten;
        if (!WriteFile(hSerial, buffer, sizeof(buffer), &bytesWritten, &ov)) {
            if (GetLastError() != ERROR_IO_PENDING) {
                PrintError(L"Ошибка отправки данных");
                break;
            }
            WaitForSingleObject(ov.hEvent, INFINITE);
        }

        Sleep(30); // Задержка для следующего опроса
    }

    // Очистка ресурсов
    CloseHandle(ov.hEvent);
    CloseHandle(hSerial);
    FreeLibrary(hXInput);
    wcout << L"Программа завершена.\n";
    return 0;
}

////#include <iostream>
////#include <windows.h>
////#include <string>
////
////int main() {
////     wcout << L"Проверка доступности COM-портов:\n";
////
////    for (int i = 1; i <= 256; i++) {
////        // Формируем имя порта
////         wstring portName = L"COM" +  to_wstring(i);
////        HANDLE hComm = CreateFile(portName.c_str(),
////            GENERIC_READ | GENERIC_WRITE,
////            0,
////            NULL,
////            OPEN_EXISTING,
////            0,
////            NULL);
////        if (hComm != INVALID_HANDLE_VALUE) {
////             wcout << L"Доступен: " << portName <<  endl;
////            CloseHandle(hComm);
////        }
////    }
////    return 0;
////}
