# winds-aloft

Get Winds and Temps Aloft data from api.weather.gov

[![NPM Version][npm-image]][npm-url]
[![Build Status][build-image]][build-url]

## Installation

```sh
npm install --save @faa-aviation-data-portal/winds-aloft
```

## Usage

```js
const winds = require('@faa-aviation-data-portal/winds-aloft')

winds
  .FD1({
    location: 'AK1',
    issuanceTimeFrom: new Date('2019-09-06T13:36:00+00:00')
  })
  .then(result => {
    console.log(JSON.stringify(result, null, 2))
  })
```

**Partial Output**:

```json
[
  {
    "id": "6e5915da-ab53-44dd-8675-8915d8136b6e",
    "wmoCollectiveId": "FBAK31",
    "issuingOffice": "KWNO",
    "issuanceTime": "2019-09-06T13:52:00+00:00",
    "productCode": "FD1",
    "productName": "6 Hour Winds Aloft Forecast",
    "productText": "\n000\nFBAK31 KWNO 061352\nFD1AK1\nDATA BASED ON 061200Z    \nVALID 061800Z   FOR USE 1400-2100Z. TEMPS NEG ABV 24000\n\nFT  3000    6000    9000   12000   18000   24000  30000  34000  39000\nADK 3222 3021-01 2919-05 3019-11 3023-23 2923-35 282350 292753 292747\nADQ 1412 1518+03 1718-01 1828-06 2032-16 2029-28 223945 245055 245559\nAKN 1629 1631-01 1735-04 1927-07 1933-18 1838-30 205247 226056 224555\nANC 9900 1607+03 1813+00 2024-05 2032-19 2225-30 243547 243257 242860\nANN 9900 1806+06 1913+01 1814-03 2022-15 2128-26 214342 214652 214759\nBET 1736 1626+00 1522-04 1519-08 1625-19 1831-31 184849 194258 203752\nBRW 1723 1826+01 1923-05 2020-10 2022-23 2023-37 222352 232558 222249\nBTI 9900 9900+02 1908-04 2210-09 2416-22 2523-35 242651 232758 232052\nBTT 1912 2122+00 2222-05 2220-09 2323-21 2228-35 232750 233058 232954\nCDB 1823 1828+03 1832-02 1734-07 1727-20 1969-30 179445 189854 205650\nCZF 1729 1623+02 1619-03 1617-09 1826-21 1835-34 183850 183557 193051\nEHM 1535 1533+00 1630-04 1728-07 1828-19 1742-31 185349 195655 194151\nFAI 1610 1414+03 1712-02 1913-07 2220-21 2322-33 253349 253357 252956\nFYU 1805 2205+02 2511-03 2415-08 2418-21 2521-34 252551 252558 252154\nGAL 2126 1819+00 1621-04 1626-10 1922-21 2030-32 224149 224358 213255\nGKN 1607 1510+04 1812+00 2114-05 2322-19 2525-32 272948 283358 272457\nHOM 1305 1612+02 1718-02 1825-06 1842-19 2038-28 213646 224257 233158\nJNU 0407 0712+06 0817-01 0617-05 3606-17 2618-29 243046 233655 243255\nLUR 2112 2118-01 2120-06 2124-11 2126-24 2135-36 214251 213955 212849\nMCG 1713 1726+00 1723-06 1728-10 1927-19 2032-31 203948 213858 213457\nMDO 9900 1506+05 1508+01 1510-03 2011-18 2412-30 282147 292456 272559\nOME 1922 1719+01 1721-04 1820-10 1921-23 2033-35 203849 194058 202952\nORT      1609+04 9900-01 2806-05 2716-20 2719-33 282550 282658 282454\nOTZ 1815 1918+01 1920-04 1924-11 2031-22 2137-35 214050 214157 212951\nSNP 9900 9900+01 9900-04 1406-10 1312-23 1422-36 143450 152551 181946\nTKA 9900 1708+01 1916-02 2021-06 2128-19 2328-30 233547 243557 233060\nUNK 1819 1617+00 1618-05 1622-10 1820-20 1927-33 204049 204358 203153\nYAK 9900 0910+06 0807+01 0806-04 3505-18 3314-32 301249 271656 262053\nIKO 2619 2619+01 2616-05 2717-11 2718-23 2618-36 241950 222547 222544\nAFM 1921 1922+02 2023-03 2023-10 2033-23 2238-35 224451 223957 212852\n5AB 2310 2510-01 2609-05 2610-09 2817-21 2823-34 285148 286753 295153\n5AC 1517 1616+07 1817+02 1823-04 1921-15 1928-27 194543 195552 203654\n5AD 0722 0913+06 0911+01 0807-03 3612-15 3428-27 334144 315552 315357\n5AE 1631 1628+03 1730-02 1822-04 2025-16 2130-28 227143 239953 258757\n5AF 0806 9900+05 9900+01 9900-04 9900-16 2010-29 202246 202254 222555\n5AG 0807 1010+05 0811+02 0709-04 0512-17 0212-31 330848 311155 271753\n",
    "parsedProductText": {
      "header": {
        "wmoCollectiveId": "FBAK31",
        "issuingOffice": "KWNO",
        "issuanceSeries": "061352",
        "productCode": "FD1",
        "productLocation": "AK1",
        "dataBasedOn": "2019-09-06T12:00:00.000Z",
        "timeValid": "2019-09-06T18:00:00.000Z",
        "timeFrom": "2019-09-06T14:00:00.000Z",
        "timeTo": "2019-09-06T21:00:00.000Z"
      },
      "data": {
        "ADK": {
          "3000": {
            "windDirectionDegrees": 320,
            "windSpeedKnots": 22,
            "tempC": null
          },
          "6000": {
            "windDirectionDegrees": 300,
            "windSpeedKnots": 21,
            "tempC": -1
          },
          "9000": {
            "windDirectionDegrees": 290,
            "windSpeedKnots": 19,
            "tempC": -5
          }
          ...
        },
        ...
      }
    }
  }
]
```

## License

MIT © [Forrest Desjardins](https://github.com/fdesjardins)

[build-url]: https://github.com/FAA-Aviation-Data-Portal/winds-aloft/actions?query=workflow%3A%22build%22
[build-image]: https://github.com/FAA-Aviation-Data-Portal/winds-aloft/workflows/build/badge.svg?branch=master&style=flat
[npm-url]: https://www.npmjs.com/package/@faa-aviation-data-portal/winds-aloft
[npm-image]: https://img.shields.io/npm/v/@faa-aviation-data-portal/winds-aloft.svg?style=flat
