# ax-bridge — интерфейсы

Договор этого проекта с платой и прошивками живёт в репозитории
[DigiD-D2](https://github.com/DrModd/DigiD-D2), файл `INTERFACES.md`. Здесь — только ссылки, чтобы
не держать две копии.

- **Выводы FPGA на плате D2** — общий раздел для ax-bridge и ax-bridge2:
  [INTERFACES.md, ветка `claude/d2-4phase`, раздел 1 «Выводы FPGA на плате D2»](https://github.com/DrModd/DigiD-D2/blob/claude/d2-4phase/INTERFACES.md#1-stm32--ax-bridge-fpga).
  Это источник раскладки: `gowin/ax_bridge_d2.cst` повторяет его и одинаков байт в байт в
  ax-bridge и ax-bridge2. Любое изменение вывода — сначала туда и в схему NAXF, потом в
  оба проекта. Вывод, который не подходит (банк, нет клокового входа, конфликт), не
  переносится молча — сначала правится схема и INTERFACES.md.
- **Регистры I2C (адрес 0x5A), AUDIO_ON, RECONFIG_N, загрузка эквалайзера** — там же,
  раздел 1.
- Текущая версия этого проекта: VERSION = 0x09 (регистр 0x06).
