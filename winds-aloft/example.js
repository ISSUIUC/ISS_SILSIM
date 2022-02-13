const winds = require('./index')

winds
  .FD1({
    location: 'AK1',
    issuanceTimeFrom: new Date('2019-09-06T13:36:00+00:00')
  })
  .then(result => {
    console.log(JSON.stringify(result, null, 2))
  })
