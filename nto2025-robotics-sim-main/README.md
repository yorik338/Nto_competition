# NTO2025 Robotics sim

Чтобы запустить модель, вам необходимо скачать последнюю версию Webots с официального сайта
https://cyberbotics.com/#download

В симуляторе выберите `File->Open World...` и откройте модель `worlds/NTO_v1.wbt`.

Чтобы начать программировать, найдите робота `ROBBO_1` в списке слева и кликните правой кнопкой мыши, затем выберите `Edit Controller`.
Аналогично проверьте контроллеры для:
1. `ROBBO_2`
2. `cam`

Скорее всего вам понадобится установить библиотеку `opencv-python` для работы с изображением.

Для удобства написания кода в VS code в папке `.vscode/settings.json` нужно изменить строку в поле `python.analysis.extraPaths` с учетом места установки симулятора Webots
- Для Linux путь скорее всего `/usr/local/webots/lib/controller/python/`
- Для Windows например `D:\\weboots\\lib\\controller\\python\\`