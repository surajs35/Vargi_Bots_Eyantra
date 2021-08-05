function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  var sheet = ss.getSheetByName(e.parameter["id"]);

  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];

  var lastRow = sheet.getLastRow();

  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();
  
  if (e.parameter["id"] == "Dashboard" && e.parameter["Order Dispatched"] == "YES"){
    for (var i = 1; i <= lastRow; i++) {
      if (sheet.getRange(i,3).getValue() == e.parameter["Order ID"]){
        cell.offset(i-1,9).setValue(e.parameter["Order Dispatched"]);
        cell.offset(i-1,12).setValue(e.parameter["Dispatch Time"]); 
      }
    }
  }
  
  else if (e.parameter["id"] == "Dashboard" && e.parameter["Order Shipped"] == "YES"){
    for (var i = 1; i <= lastRow; i++) {
      if (sheet.getRange(i,3).getValue() == e.parameter["Order ID"]){
        cell.offset(i-1,10).setValue(e.parameter["Order Shipped"]);
        cell.offset(i-1,13).setValue(e.parameter["Shipping Time"]);
        cell.offset(i-1,14).setValue(e.parameter["Time Taken"]);
      }
    }
  }
  
  else{
    for (i in headers){
      
      // loop through the headers and if a parameter name matches the header name insert the value
      
      if (headers[i] == "Timestamp")
      {
        val = d.toDateString() + ", " + d.toLocaleTimeString();
      }    
      else
      {
        val = e.parameter[headers[i]]; 
      }
      
      // append data to the last row
      cell.offset(lastRow, col).setValue(val);
      col++;
      
      if(headers[i] == "Dispatch Status"){
        check = e.parameter[headers[i]];
        if (check == "YES"){
          var order = e.parameter["Order ID"];
          var item = e.parameter["Item"];
          var dis_qnty = e.parameter["Dispatch Quantity"];
          var dis_time = e.parameter["Dispatch Date and Time"];
          var city = e.parameter["City"];
          var cost = e.parameter["Cost"];
          
          var message = "Hello!"+"\n"+"Your order has been dispatched,contact us if have any questions.We are here to help you."+"\n \n"+"ORDER SUMMARY:"+"\n \n"+
            "Order Number:"+order+"\n"+
            "Item:"+item+"\n"+
            "Quantity:"+dis_qnty+"\n"+
            "Dispatched Date and Time:"+dis_time+"\n"+
            "City:"+city+"\n"+
            "Cost:"+cost+"\n"; 
          
          var to = "eyrc.vb.0441@gmail.com";   //write your email id here
          
          MailApp.sendEmail(to, " Your Order is Dispatched! ", message);
          MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Dispatched! ", message);
        }
      }
      
      if(headers[i] == "Shipped Status"){
        check = e.parameter[headers[i]];
        if (check == "YES"){
          var order = e.parameter["Order ID"];
          var item = e.parameter["Item"];
          var ship_qnty = e.parameter["Shipped Quantity"];
          var ship_time = e.parameter["Shipped Date and Time"];
          var city = e.parameter["City"];
          var cost = e.parameter["Cost"];
          var est_time = e.parameter["Estimated Time of Delivery"];
          
          var message = "Hello!"+"\n"+"Your order has been shipped.It will be drone delivered to you within estimated Time."+
            "Contact us if have any questions.We are here to help you."+"\n \n"+
            "ORDER SUMMARY:"+"\n \n"+
            "Order Number:"+order+"\n"+
            "Item:"+item+"\n"+
            "Quantity:"+ship_qnty+"\n"+
            "Shipped Date and Time:"+ship_time+"\n"+
            "City:"+city+"\n"+
            "Cost:"+cost+"\n"+
            "Estimated Time of delivery:"+est_time; 
          
          var to = "eyrc.vb.0441@gmail.com";   //write your email id here
          
          MailApp.sendEmail(to, " Your Order is Shipped! ", message);
          MailApp.sendEmail("eyrc.vb.0000@gmail.com", " Your Order is Shipped! ", message);
          
        }
      }
    }
  }

  return ContentService.createTextOutput('success');
}

