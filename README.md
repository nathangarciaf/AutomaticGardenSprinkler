# Automatic Garden Sprinkler - Irrigador Automático de Jardim

Projeto desenvolvido na disciplina de Projeto Integrado de Computação II.

Grupo: Jordano Furtado, Luana Da Ros Almeida e Nathan Garcia Freitas

## Conceito & Motivação

A disciplina em si consiste em criar uma integração completa entre Hardware e 
Software, portanto, foi decidido pelo grupo a criação de uma estrutura cujo objetivo 
é fazer uma varredura dentro das mini jardineiras que ficam entre os trilhos da estrutura, 
com isso, cada mini jardineira terá a umidade da terra medida e haverá então uma verificação 
se há a necessidade de regar a área, tudo isso baseado em configurações definidas pelo 
próprio usuário. A ideia principal do projeto é deixar todo o sistema rodando intermitentemente 
e executando de maneira periódica, ou seja, a cada determinado tempo (de hora em hora, por exemplo) 
toda a rotina seja executada, com isso, servindo como um gestor inteligente de uma pequena horta/jardim 
doméstico com capacidade de escalabilidade.

## Componentes

- ESP8266
- Sensor de umidade
- Sensor de proximidade infravermelho
- Servo Motor 360
- Servo Motor 180
- Módulo Relé
- Bomba Hidráulica 12V
- Outros componentes elétricos

## Circuito

O circuito foi desenvolvido em uma placa perfurada para evitar a grande quantidade de jumpers interligando 
os componentes. Todo o esquemático está disponível dentro da pasta esquemático.

<img width="817" height="700" alt="Captura de tela 2025-08-31 235547" src="https://github.com/user-attachments/assets/32e27338-9e1d-4409-a940-8ad3a069bf58" />

